#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <khepera/khepera.h>
#include <signal.h>
#include <math.h>

//#define DEBUG 1
#define PORT_NUMBER 50000
// #define for driver mode
#define BIG_SPEED_FACTOR 25
#define SPEED_FACTOR 1
#define MAX_SPEED 1500
#define MIN_SPEED 15
#define DEFAULT_SPEED 200
#define ROTATE_HIGH_SPEED_FACT 0.5
#define ROTATE_LOW_SPEED_FACT 0.75
#define ROT_SPEED_HIGH_TRESH 300
#define STOP_TIME 100000 // us

#define SIGN(x) ((x)>0?1:((x)<0?-1:0))  // sign or zero

static knet_dev_t * dsPic; // robot pic microcontroller access

int maxsp,accinc,accdiv,minspacc, minspdec; // for speed profile

static int quitReq = 0; // quit variable for loop

static void ctrlc_handler( int sig ) 
{
  quitReq = 1;
  
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic );
  
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy
  
  kb_change_term_mode(0); // revert to original terminal if called
  
  exit(0);
}

int main(int argc, char *argv[]) {
  // Khepera IV Init variables
  #define IR_BAR_LEN 15 	// display bar length for IR sensor
  #define US_BAR_LEN 23 	// display bar length for US sensor
  #define ACGY_BAR_LEN 30 // display bar length for Accel/gyro sensor
  #define MAX_US_DISTANCE 250.0 // max distance US
  #define MAX_G 2 		// max acceleration in g
  // convert US value to text comment
  #define US_VAL(val) ((val)==KH4_US_DISABLED_SENSOR ? "Not activated" : ((val)==KH4_US_NO_OBJECT_IN_RANGE ? "No object in range" : ((val)==KH4_US_OBJECT_NEAR ? "Object at less than 25cm" : "Object in range 25..250cm")))
  double fpos,dval,dmean;
  long lpos,rpos;
  char Buffer[100],bar[12][64],revision,version;
  int i,n,type_of_test=0,sl,sr,pl,pr;
  short index, value,sensors[12],usvalues[5];
  char c;
  long motspeed;
  char line[80],l[9];
  int kp,ki,kd;
  int pmarg;

  // Server variables
  int sockfd, newsockfd, portno;
  socklen_t clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  struct timeval rcv_tv;

  // Drive variables
  int out=0,speed=DEFAULT_SPEED,vsl,vsr,anymove=0;
  int vel[2], vel_l, vel_r;
  float v, w0;
  float xg, yg;
  char init_c = 'n';
	struct timeval startt,endt;
  float w = 0.f;
  float d = 0.f;
  float xp = 0.f;
  float yp = 0.f;
  float theta = 0.f;
  float alfa = 0.f;

  /* Set the libkhepera debug level - Highly recommended for development. */
  // kb_set_debug_level(2);

  printf("LibKhepera Robotic Park Program\r\n");
  
  // Initiate libkhepera and robot access
  if(kh4_init(argc ,argv)!=0){
  	printf("\nERROR: could not initiate the libkhepera!\n\n");
  	return -1;
  }	

  /* Open robot socket and store the handle in their respective pointers */
  dsPic = knet_open("Khepera4:dsPic",KNET_BUS_I2C,0,NULL);

	if(dsPic==NULL){
  	printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
  	return -2;
  }	

  /* initialize the motors controlers*/
  /* tuned parameters */
  pmarg=20;
  kh4_SetPositionMargin(pmarg,dsPic ); 				// position control margin
  kp=10;
  ki=5;
  kd=1;
  kh4_ConfigurePID( kp , ki , kd,dsPic  ); 		// configure P,I,D
  
  accinc=3;//3;
  accdiv=0;
  minspacc=20;
  minspdec=1;
  maxsp=400;
  // configure acceleration slope
  kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
  
	kh4_SetMode( kh4RegIdle,dsPic );  				// Put in idle mode (no control)

  // get revision
  if(kh4_revision(Buffer, dsPic)==0){
   	version=(Buffer[0]>>4) +'A';
  	revision=Buffer[0] & 0x0F; 
    printf("\r\nVersion = %c, Revision = %u\r\n",version,revision);        
  }
  
  signal( SIGINT , ctrlc_handler ); // set signal for catching ctrl-c

	/* ADD YOUR CODE HERE */
  // Init visual signal
  kh4_activate_us(0,dsPic);
  kh4_SetRGBLeds(255,0,0,255,0,0,255,0,0, dsPic);
  // Open a socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    perror("ERROR opening socket");
    exit(1);
  }

  // Clear the server address structure
  memset((char *) &serv_addr, 0, sizeof(serv_addr));

  // Set the port number
  portno = PORT_NUMBER;

  // Set the server address structure
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);

  // Bind the socket to the specified port
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    perror("ERROR on binding");
    exit(1);
  }

  // Listen for incoming connections
  listen(sockfd,5);
  clilen = sizeof(cli_addr);

  // Outer loop: accept a (new) ROS 2 client. Previously accept() was
  // called once, outside any loop -- if the ROS 2 side ever dropped and
  // reconnected (network hiccup, node restart), the robot had no way to
  // accept the new connection and stayed wedged on the dead socket until
  // this program was killed and restarted by hand on the robot. That was
  // one of the two root causes behind the intermittent "communication
  // hangs" (the other is the read() timeout added below).
  while (quitReq==0){

    printf("Waiting for ROS2 Client...\r\n");
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0) {
      if (quitReq) break;
      perror("ERROR on accept");
      continue;
    }

    // Give the client socket a receive timeout so a silent/stalled link
    // can no longer freeze the odometry/control loop below: read() now
    // returns EAGAIN/EWOULDBLOCK on a quiet socket instead of blocking
    // forever. 50 ms bounds how stale the control loop can get; tune if
    // needed once verified on the real robot.
    rcv_tv.tv_sec = 0;
    rcv_tv.tv_usec = 50000;
    setsockopt(newsockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *) &rcv_tv, sizeof(rcv_tv));

    // Ready visual signal
    kh4_ResetEncoders(dsPic);
    kh4_SetMode(kh4RegSpeed,dsPic );
    kh4_SetRGBLeds(0,0,255,0,0,255,0,0,255, dsPic);

    printf("Connected to ROS2 Client\r\n");
    init_c = 'n'; // fresh session: no goal-tracking active until a 'g' arrives

    // Inner loop: serve this client until it disconnects or errors out,
    // then fall back to the outer loop to accept the next one.
    while (quitReq==0){

      kh4_get_position(&pl,&pr,dsPic);
      kh4_ResetEncoders(dsPic);
      d = (pr + pl)*KH4_PULSE_TO_MM/2000;
      w = (pr - pl)*KH4_PULSE_TO_MM/105;
      xp = xp + d*cos(theta);
      yp = yp + d*sin(theta);
      theta = theta + w;
      printf("Xp: %.3f  Yp %.3f\n", xp, yp);
      if(init_c == 'y'){
        v = ((xg-xp)*cos(theta) + (yg-yp)*sin(theta))*10;
        if(v<0.02 && v>-0.02){
          v = 0.0;
        }
        alfa = atan2(yg-yp, xg-xp);
        w0 = 1 * sin(alfa-theta);
        printf("onboard cmd: %.3f %.3f \n", v, w0);
        vel_r = (int)((v+w0*0.5*10.54)/(KH4_SPEED_TO_MM_S/10));
        vel_l = (int)((v-w0*0.5*10.54)/(KH4_SPEED_TO_MM_S/10));
        kh4_set_speed(vel_l, vel_r,dsPic );
      }
      // Clear the buffer
      memset(buffer, 0, 256);

      // Read data from the client (bounded by SO_RCVTIMEO above)
      n = read(newsockfd, buffer, 255);
      if (n < 0){
        if (errno == EAGAIN || errno == EWOULDBLOCK){
          // No command arrived this cycle -- not an error, keep controlling.
          continue;
        }
        perror("ERROR reading from socket");
        break; // drop this client, go back to accept()
      }
      if (n == 0){
        // Client closed the connection cleanly.
        printf("ROS2 Client disconnected\r\n");
        break;
      }

      // Check the command received from the client
      if (buffer[0]=='d'){
        sscanf(buffer,"%*c %f %f",&v,&w0);
        printf("cmd: %.3f %.3f \n", v, w0);
        vel_r = (int)((v+w0*0.5*10.54)/(KH4_SPEED_TO_MM_S/10));
        vel_l = (int)((v-w0*0.5*10.54)/(KH4_SPEED_TO_MM_S/10));
        // Only drive from cmd_vel when there is no active goal-tracking
        // session (see the init_c=='y' block above): letting both write
        // to the motors in the same cycle would fight each other. This
        // call used to be commented out entirely, so cmd_vel never moved
        // the physical robot at all -- verify this precedence on real
        // hardware before relying on it operationally.
        if (init_c != 'y'){
          kh4_set_speed(vel_l, vel_r,dsPic );
        }
      }
      if (buffer[0]=='g'){
        sscanf(buffer,"%*c %f %f",&xg,&yg);
        init_c = 'y';
        printf("Goal Pose: %.3f %.3f \n", xg, yg);
      }
      if (buffer[0]=='i'){
        sscanf(buffer,"%*c %f %f",&xp,&yp);
        printf("Pose: %.3f %.3f \n", xp, yp);
      }
      else if (strcmp(buffer, "stop") == 0){
        printf("Stop move\n");
        kh4_set_speed(0 ,0,dsPic );
      }
      else if (buffer[0]=='p'){
        // Send the data string to the client
        char data_str[256];
        sprintf(data_str, "%.4f,%.4f", d,theta);
        write(newsockfd, data_str, strlen(data_str));
      }
      else if (strcmp(buffer, "reset_pose") == 0){
        sscanf(buffer,"%*c %d %d",&sl,&sr);
        kh4_SetMode(kh4RegPosition,dsPic );
        kh4_set_position(sl,sr, dsPic);
      }
      else if (strcmp(buffer, "get_data") == 0){
        // Read data from the sensors
        // get and print us sensors
        kh4_measure_us(Buffer,dsPic);
        for (i=0;i<5;i++){
          usvalues[i] = (short)(Buffer[i*2] | Buffer[i*2+1]<<8);
        }

        printf("\nUS sensors : distance [cm]\
              \nleft 90   : %4d\
              \nleft 45   : %4d\
              \nfront     : %4d\
              \nright 45  : %4d\
              \nright 90  : %4d\n", usvalues[0],usvalues[1],usvalues[2],usvalues[3],usvalues[4]);
        usleep(20000); // wait 20ms

        // Convert the sensor data to a string
        char data_str[256];
        sprintf(data_str, "%d,%d,%d,%d,%d", usvalues[0], usvalues[1], usvalues[2], usvalues[3], usvalues[4]);

        // Send the data string to the client
        write(newsockfd, data_str, strlen(data_str));
      }

    }

    // This client is gone: stop the robot for safety and close its
    // socket before going back to accept() for the next one.
    kh4_set_speed(0 ,0 ,dsPic);
    kh4_SetMode( kh4RegIdle,dsPic );
    close(newsockfd);
  }

  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy

  // Close the socket
  close(sockfd);

 return 0;
}


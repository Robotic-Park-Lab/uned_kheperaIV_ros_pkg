/******************************************************************************
 * Código optimizado para Khepera IV - Comunicación ROS2
 * 
 * Características principales:
 * - Control de motores con interpolación suave
 * - Comunicación TCP no bloqueante
 * - Odometría precisa
 * - Seguridad con mutex para datos compartidos
 * - Fácil integración con ROS2
 * 
 * Autor: Francisco José Mañas Álvarez
 * Lab: Robotic Park
 * Fecha: 05-2025
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <khepera/khepera.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>

// ======================= CONSTANTES DE CONFIGURACIÓN ========================
//#define DEBUG 1
#define PORT_NUMBER 50000          // Puerto TCP para comunicación
#define BUF_SIZE 1024              // Tamaño del buffer circular
#define CONTROL_LOOP_HZ 100        // Frecuencia del loop de control (Hz)
#define MAX_SPEED 1500             // Velocidad máxima del motor (pulsos/seg)
//#define KH4_PULSE_TO_MM 0.0041f    // Conversión de pulsos a milímetros
//#define KH4_SPEED_TO_MM_S 41.0f    // Conversión de velocidad
#ifndef CLOCK_MONOTONIC
    #define CLOCK_MONOTONIC 1  // Valor estándar en POSIX
#endif

// ======================= ESTRUCTURAS DE DATOS ===============================
typedef struct {
  float x, y, theta;         // Pose actual del robot (mm, mm, rad)
  float rx, ry, rtheta;      // Goal Pose del robot (mm, mm, rad)
  float v, w;                // Velocidad lineal (mm/s) y angular (rad/s)
  float rv, rw;              // Ref: Velocidad lineal (mm/s) y angular (rad/s)
  int vel_l, vel_r;          // Ref: Velocidades de los motores (pulsos/seg)
  int quit;                  // Bandera para terminar el programa
  int cmd_mode;              // 0: Open Loop, 1: Close Loop, 2: Rele
  pthread_mutex_t mutex;     // Mutex para acceso seguro a datos compartidos
} SharedData;

typedef struct {
  float ax, ay, az;         // Aceleración lineal
  float wx, wy, wz;         // aceleración angular
  float u[5];                // Ultrasonidos
} IMU;

typedef struct {
  long cmd;         // 
  float threshold;         // aceleración angular
  int level;                // Ultrasonidos
} Relay;
// ======================= VARIABLES GLOBALES =================================
static knet_dev_t *dsPic;      // Dispositivo del controlador Khepera
SharedData shared;             // Datos compartidos entre hilos
IMU sensor_data;
Relay rele_controller;
int sockfd, newsockfd;         // Descriptores de socket
int code;
int rele;
int j;
char log_filename[256];        // Variable para el nombre del archivo de log
FILE *log_file = NULL;
// ======================= PROTOTIPOS DE FUNCIONES ============================
void init_khepera();
void setup_tcp_server();
void* motor_control_thread(void* arg);
void* tcp_communication_thread(void* arg);
void process_message(const char* msg);
void ctrlc_handler(int sig);
void init_log_file();          //

// ========================= DRIVER CONFIGURATION =============================
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

void ctrlc_handler(int sig){
  printf("\nRecibida señal Ctrl+C. Terminando...\n");
  shared.quit = 1;
  
  kh4_set_speed(0 ,0 ,dsPic);               // stop robot
  kh4_SetMode( kh4RegIdle,dsPic );
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic);  // clear rgb leds because consumes energy
  kb_change_term_mode(0);                   // revert to original terminal if called
  fclose(log_file);
  printf("Datos guardados en: %s\n", log_filename);
  exit(0);
}

void init_log_file() {
  // Generar nombre de archivo con timestamp
  time_t now = time(NULL);
  struct tm *tm = localtime(&now);
  snprintf(log_filename, sizeof(log_filename), 
           "/home/root/%04d%02d%02d_%02d%02d%02d.txt",
           tm->tm_year + 1903, tm->tm_mon + 3, tm->tm_mday-11,
           tm->tm_hour+4, tm->tm_min-13, tm->tm_sec);
  
  // Crear archivo con encabezado
  log_file = fopen(log_filename, "w");
  if (log_file != NULL) {
      fprintf(log_file, "Timestamp_us,V_mm/s,V_r_mm/s,Acc_g\n"); //,W_rad/s,W_r_rad/s\n");
      fclose(log_file);
      printf("Archivo de registro creado: %s\n", log_filename);
  } else {
      perror("Error al crear archivo de registro");
  }
}

// ======================= FUNCIONES DE INICIALIZACIÓN ========================
void init_khepera() {
  printf("Inicializando Khepera IV...\n");
  code = rele = 0;
  shared.cmd_mode = 0;
  rele_controller.cmd = KH4_MAX_OPENLOOP_SPEED_PWM_100*0.9f;  //
  rele_controller.threshold = 150.0f;
  j = 0;
  if (kh4_init(0, NULL) != 0) {
      fprintf(stderr, "Error: No se pudo inicializar libkhepera\n");
      exit(1);
  }

  dsPic = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);
  if (!dsPic) {
      fprintf(stderr, "Error: No se pudo abrir comunicación con el dsPic\n");
      exit(1);
  }
  kh4_activate_us(0,dsPic);
  kh4_SetRGBLeds(255,0,0,255,0,0,255,0,0, dsPic);
  kh4_SetPositionMargin(20,dsPic ); 				// position control margin
  // Configuración de control PID para los motores
  //kh4_ConfigurePID(10, 5, 1, dsPic);            // Kp=10, Ki=5, Kd=1
  //kh4_ConfigurePID(1.014, 7.043, 0, dsPic);            // Kp=10, Ki=5, Kd=1
  //kh4_ConfigurePID(1.626, 7.199, 0, dsPic);
  kh4_ConfigurePID(1.626*2.0, 7.199*2.0, 0, dsPic);
  kh4_SetSpeedProfile(3, 0, 20, 1, 400, dsPic);  // Perfil de velocidad
  kh4_SetMode(kh4RegSpeed, dsPic);               // Modo de velocidad
  
  printf("Khepera IV inicializado correctamente.\n");

  /*
  if(kh4_revision(Buffer, dsPic)==0){
   	version=(Buffer[0]>>4) +'A';
  	revision=Buffer[0] & 0x0F; 
    printf("\r\nVersion = %c, Revision = %u\r\n",version,revision);        
  }
  */
}

void setup_tcp_server() {
  struct sockaddr_in serv_addr;
  
  printf("Iniciando servidor TCP en puerto %d...\n", PORT_NUMBER);
  
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
      perror("Error al abrir el socket");
      exit(1);
  }

  // Configuración del socket no bloqueante
  fcntl(sockfd, F_SETFL, O_NONBLOCK);

  int opt = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(PORT_NUMBER);

  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
      perror("Error en bind");
      exit(1);
  }

  listen(sockfd, 5);
  printf("Servidor TCP listo. Esperando conexiones...\n");

  /*
  clilen = sizeof(cli_addr);
  // Accept an incoming connection
  newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
  if (newsockfd < 0) {
    perror("ERROR on accept");
    exit(1);
  }
  */
}

// ======================= HILO DE CONTROL DE MOTORES =========================
void* motor_control_thread(void* arg) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  long last_time = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
  int prev_vel_l = 0, prev_vel_r = 0;

  printf("Hilo de control de motores iniciado (%dHz)\n", CONTROL_LOOP_HZ);
  
  while (!shared.quit) {
      clock_gettime(CLOCK_MONOTONIC, &ts);
      // long now = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
      // float dt = (now - last_time) / 1000000.0f;
      // last_time = now;
      /*
      pthread_mutex_lock(&shared.mutex);
      int target_vel_l = shared.vel_l;
      int target_vel_r = shared.vel_r;
      pthread_mutex_unlock(&shared.mutex);
      */
      // Interpolación suave de velocidades (rampa)
      /*
      if (target_vel_l != prev_vel_l || target_vel_r != prev_vel_r) {
          int step_l = (target_vel_l - prev_vel_l) / 4;
          int step_r = (target_vel_r - prev_vel_r) / 4;
          
          step_l = (step_l == 0) ? (target_vel_l > prev_vel_l ? 1 : -1) : step_l;
          step_r = (step_r == 0) ? (target_vel_r > prev_vel_r ? 1 : -1) : step_r;
          
          prev_vel_l += step_l;
          prev_vel_r += step_r;
      }
      */
      kh4_set_speed(shared.vel_l, shared.vel_r, dsPic);
      usleep(1000000 / CONTROL_LOOP_HZ);  // Control a frecuencia constante
  }
  
  printf("Hilo de control de motores terminado.\n");
  return NULL;
}

// ======================= HILO DE COMUNICACIÓN TCP ===========================
void* tcp_communication_thread(void* arg) {
  char buffer[BUF_SIZE];
  int n;
  fd_set readfds;
  struct timeval timeout;

  printf("Hilo de comunicación TCP iniciado\n");
  
  while (!shared.quit) {
      socklen_t clilen = sizeof(struct sockaddr_in);
      struct sockaddr_in cli_addr;
      
      FD_ZERO(&readfds);
      FD_SET(sockfd, &readfds);
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;
      
      int ready = select(sockfd+1, &readfds, NULL, NULL, &timeout);
      if (ready < 0) {
          perror("Error en select");
          continue;
      }
      
      if (FD_ISSET(sockfd, &readfds)) {
          newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
          if (newsockfd < 0) {
              if (errno != EWOULDBLOCK) {
                  perror("Error en accept");
              }
              continue;
          }

          printf("Cliente conectado\n");
          kh4_SetRGBLeds(0,255,0,0,255,0,0,255,0, dsPic);
          fcntl(newsockfd, F_SETFL, O_NONBLOCK);

          // Bucle de comunicación con el cliente
          while (!shared.quit) {
              FD_ZERO(&readfds);
              FD_SET(newsockfd, &readfds);
              timeout.tv_sec = 0;
              timeout.tv_usec = 100000;  // 100ms timeout
              
              ready = select(newsockfd+1, &readfds, NULL, NULL, &timeout);
              if (ready < 0) {
                  perror("Error en select");
                  break;
              }
              
              if (FD_ISSET(newsockfd, &readfds)) {
                  n = read(newsockfd, buffer, BUF_SIZE - 1);
                  if (n < 0) {
                      if (errno != EWOULDBLOCK) {
                          perror("Error en read");
                          break;
                      }
                  } else if (n == 0) {
                      printf("Cliente desconectado\n");
                      kh4_SetRGBLeds(255,0,0,255,0,0,255,0,0, dsPic);
                      pthread_mutex_lock(&shared.mutex);
                      shared.vel_l = shared.vel_r = 0;
                      pthread_mutex_unlock(&shared.mutex);
                      break;
                  } else {
                      buffer[n] = '\0';
                      process_message(buffer);
                  }
              }
          }

          close(newsockfd);
      }
  }
  
  printf("Hilo de comunicación TCP terminado.\n");
  return NULL;
}

// ======================= PROCESAMIENTO DE MENSAJES ==========================
void process_message(const char* msg) {
  float v, w, x, y;
  if (sscanf(msg, "d %f %f", &v, &w) == 2) {
    if(shared.rv == 0.0f){
      log_file = fopen(log_filename, "a");
      printf("Recording ...\n");
    }else if(shared.rv == 0.0f && log_file != NULL){
      fclose(log_file);
      log_file = NULL;
      printf("Record stopped\n");
    }
    
      // Comando de velocidad (v, w)
      shared.cmd_mode = 0;
      pthread_mutex_lock(&shared.mutex);
      shared.rv = v;
      shared.rw = w;
      shared.vel_l = (int)((v - w * 0.5 * 10.54) / (KH4_SPEED_TO_MM_S / 10));
      shared.vel_r = (int)((v + w * 0.5 * 10.54) / (KH4_SPEED_TO_MM_S / 10));
      pthread_mutex_unlock(&shared.mutex);
      printf("Comando velocidad: v=%.2f mm/s, w=%.2f rad/s\n", v, w);
      code = 1;
  }
  else if (sscanf(msg, "g %f %f", &x, &y) == 2) {
      // Comando de posición objetivo (x, y)
      shared.cmd_mode = 1;
      pthread_mutex_lock(&shared.mutex);
      shared.rx = x;
      shared.ry = y;
      pthread_mutex_unlock(&shared.mutex);
      
      printf("Nuevo objetivo: x=%.2f mm, y=%.2f mm\n", x, y);
      code = 2;
      kh4_SetRGBLeds(0,0,255,0,0,255,0,0,255, dsPic);
  }
  else if (sscanf(msg, "i %f %f", &x, &y) == 2) {
    // Comando de posición objetivo (x, y)
    shared.cmd_mode = 0;
    pthread_mutex_lock(&shared.mutex);
    shared.x = x;
    shared.y = y;
    pthread_mutex_unlock(&shared.mutex);
    
    printf("Posición inicializada: x=%.2f mm, y=%.2f mm\n", x, y);
    code = 3;
  }
  else if (strcmp(msg, "stop") == 0) {
      // Detener el robot
      pthread_mutex_lock(&shared.mutex);
      shared.vel_l = shared.vel_r = 0;
      pthread_mutex_unlock(&shared.mutex);
      
      printf("Comando STOP recibido\n");
      code = 4;
  }
  else if (strcmp(msg, "p") == 0) {
      // Enviar pose actual
      char response[256];
      pthread_mutex_lock(&shared.mutex);
      sprintf(response, "%.2f,%.2f,%.2f", shared.x, shared.y, shared.theta);
      pthread_mutex_unlock(&shared.mutex);
      
      write(newsockfd, response, strlen(response));
      if(code != 5){
        printf("Enviando pose actual ...\n");
      }
      code = 5;
  }
  else if (strcmp(msg, "r") == 0) {
    // Experiencia relé
    if (shared.cmd_mode == 2){
      printf("Relé Off\n");
      fclose(log_file);
      log_file = NULL;
      kh4_SetRGBLeds(0,255,0,0,255,0,0,255,0, dsPic);
      kh4_SetPositionMargin(20,dsPic ); 				// position control margin
      // Configuración de control PID para los motores
      kh4_ConfigurePID(10, 5, 1, dsPic);            // Kp=10, Ki=5, Kd=1
      kh4_SetSpeedProfile(3, 0, 20, 1, 400, dsPic);  // Perfil de velocidad
      kh4_SetMode(kh4RegSpeed, dsPic);               // Modo de velocidad
      shared.cmd_mode = 0;
      shared.rv = shared.rw = 0;
      shared.vel_l = shared.vel_r = 0;
    }else{
      printf("Relé On\n");
      log_file = fopen(log_filename, "a");
      kh4_SetRGBLeds(0,0,255,0,0,255,0,0,255, dsPic);
      kh4_SetMode(kh4RegSOpenLoop,dsPic );          // Modo Open Loop
      shared.cmd_mode = 2;
    }
    code = 6;
  }
  else if (strcmp(msg, "s") == 0) {
      // gyroscope sensor
      char Buffer[100];
      int i;
      double dval;
      float aux = 0.0f;
      kh4_measure_gyro((char *)Buffer, dsPic);
      for (i=0;i<3;i++)
      {
        aux += ((short)((Buffer[i*2] | Buffer[i*2+1]<<8)))*KH4_GYRO_DEG_S; // convert to [deg/s]
      }
      sensor_data.wx = aux/3;
      aux = 0.0f;
      for (i=10;i<13;i++)
      {
        aux += ((short)((Buffer[i*2] | Buffer[i*2+1]<<8)))*KH4_GYRO_DEG_S; // convert to [deg/s]
      }
      sensor_data.wy = aux/3;
      aux = 0.0f;
      for (i=20;i<23;i++)
      {
        aux += ((short)((Buffer[i*2] | Buffer[i*2+1]<<8)))*KH4_GYRO_DEG_S; // convert to [deg/s]
      }
      sensor_data.wz = aux/3;
      aux = 0.0f;
      // printf("\nGyro: %.2f %.2f %.2f", sensor_data.wx, sensor_data.wy, sensor_data.wz);
    

      // get accel sensor
		  kh4_measure_acc((char *)Buffer, dsPic);
      for (i=0;i<3;i++)
      {
        aux += ((short)(Buffer[i*2] | Buffer[i*2+1]<<8)>>4)/1000.0; // convert to [g]
      }
      sensor_data.ax = aux/3;
      aux = 0.0f;
      for (i=10;i<13;i++)
      {
        aux += ((short)(Buffer[i*2] | Buffer[i*2+1]<<8)>>4)/1000.0; // convert to [g]
      }
      sensor_data.ay = aux/3;
      aux = 0.0f;
      for (i=20;i<23;i++)
      {
        aux += ((short)(Buffer[i*2] | Buffer[i*2+1]<<8)>>4)/1000.0; // convert to [g]
      }
      sensor_data.az = aux/3;
      aux = 0.0f;

      // printf("\nAcc: %.2f %.2f %.2f", sensor_data.ax, sensor_data.ay, sensor_data.az);

      char response[256];
      sprintf(response, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", sensor_data.ax, sensor_data.ay, sensor_data.az, sensor_data.wx, sensor_data.wy, sensor_data.wz);
      
      write(newsockfd, response, strlen(response));
      // if(code != 6){printf("\nEnviando datos de sensores ...\n");}
      // code = 7;
  }
  else {
      printf("Comando no reconocido: %s\n", msg);
      /*
      else if (buffer[0]=='t'){
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
      */
  }
}

/*######################################################################################*/
int main(int argc, char *argv[]) {
  printf("\n=== Sistema de Control Khepera IV - ROS2 Interface ===\n");
  
  // Configurar handler para Ctrl+C
  signal(SIGINT, ctrlc_handler);

  // Inicializar archivo de registro
  init_log_file();

  // Inicializar estructura compartida
  memset(&shared, 0, sizeof(SharedData));
  pthread_mutex_init(&shared.mutex, NULL);
  shared.quit = 0;

  // Inicializar hardware y comunicación
  init_khepera();
  setup_tcp_server();

  // Crear hilos de ejecución
  pthread_t motor_thread, tcp_thread;
  pthread_create(&motor_thread, NULL, motor_control_thread, NULL);
  pthread_create(&tcp_thread, NULL, tcp_communication_thread, NULL);
  

  // Ready visual signal
  kh4_ResetEncoders(dsPic);
  kh4_SetMode(kh4RegSpeed,dsPic );
  kh4_SetRGBLeds(0,255,0,0,255,0,0,255,0, dsPic);

  printf("Connected to ROS2 Client. Esperando comandos ...\r\n");
  // Bucle principal - Actualización de odometría

  // Main loop
  struct timespec ts;
  long now = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
  while (!shared.quit){
    int lpos, rpos, vsl,vsr;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    now = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    kh4_get_position(&lpos, &rpos, dsPic);
    kh4_get_speed(&vsl,&vsr,dsPic );
    kh4_ResetEncoders(dsPic);
        
    pthread_mutex_lock(&shared.mutex);
    // Cálculo de odometría (posición y orientación)
    float delta_theta = (rpos - lpos) * KH4_PULSE_TO_MM / KH4_WHEELS_DISTANCE;
    float delta_pos = (rpos + lpos) * KH4_PULSE_TO_MM / 2000.0f;
    
    shared.v = (vsl+vsr)*KH4_SPEED_TO_MM_S/2;
    shared.w = (vsr-vsl)*KH4_SPEED_TO_MM_S/KH4_WHEELS_DISTANCE;
    shared.theta += delta_theta;
    shared.x += delta_pos * cos(shared.theta);
    shared.y += delta_pos * sin(shared.theta);
    pthread_mutex_unlock(&shared.mutex);

    //printf("Xp: %.3f  Yp %.3f\n", shared.x, shared.y);
    if(shared.cmd_mode == 0){
      if (log_file != NULL) {
        fprintf(log_file, "%ld,%.3f,%.3f,%.3f\n", 
          now, shared.v, shared.rv*100.0f/KH4_MAX_OPENLOOP_SPEED_PWM_100, sensor_data.ax);
      }  
    }
    if(shared.cmd_mode == 1){
      float d_error = (shared.rx-shared.x)*cos(shared.theta) + (shared.ry-shared.y)*sin(shared.theta);
      float v, w0;
      // d_error = sqrt(pow(xg-xp,2))+(pow(yg-yp,2))*100;
      printf("D: %.3f \t cmd: %.3f %.3f \n", d_error, v, w0);
      if(d_error>0.01 || d_error<-0.01){
        float alfa = atan2(shared.ry-shared.y, shared.rx-shared.x);
        v = (d_error)*10;
        w0 = 1 * sin(alfa-shared.theta);
        /*
        oc = alfa - theta;
        eo = atan2(sin(oc),cos(oc));
        p = (3.14-abs(eo))/3.14;
        v = 1.0*d_error*p;
        if(v>10.0){
          v = 10.0;
        }
        eomas = eo + eomas;
        w0 = 1.5*sin(eo) + 0.008*eomas*0.003;
        */      
      } else {
        v = 0.0;
        w0 = 0.0;
        kh4_SetRGBLeds(0,255,0,0,255,0,0,255,0, dsPic);
      }
      pthread_mutex_lock(&shared.mutex);
      shared.rv = v;
      shared.rw = w0;
      shared.vel_l = (int)((shared.rv - shared.rw * 0.5 * 10.54) / (KH4_SPEED_TO_MM_S / 10));
      shared.vel_r = (int)((shared.rv + shared.rw * 0.5 * 10.54) / (KH4_SPEED_TO_MM_S / 10));
      pthread_mutex_unlock(&shared.mutex);
      if (log_file != NULL) {
        fprintf(log_file, "%ld,%.3f,%.3f, %.3f\n", 
          now, shared.v, shared.rv*100.0f/KH4_MAX_OPENLOOP_SPEED_PWM_100, sensor_data.ax);
      }     
    }
    else if(shared.cmd_mode == 2){
      /*
      // Open loop
      motspeed= KH4_MAX_OPENLOOP_SPEED_PWM_100/4;  // 25%
      kh4_SetMode(kh4RegSOpenLoop,dsPic );
      kh4_set_speed(-motspeed ,motspeed ,dsPic);
      printf("\nRotating (control pwm: %ld %%) with openloop during 3s\n",motspeed*100/KH4_MAX_OPENLOOP_SPEED_PWM_100);
      
      */
      /*
      int kh4_set_speed(int left,int right, knet_dev_t *hDev) (units: encoder)
      currentRegType==kh4RegSOpenLoop
      // openloop : 2940 units = 100 %)
			return kh4_sendcommand(hDev,I2C_MOT_LEFT_PWM_CONSIGN_L,outbuf,4);
      
      */
      shared.rw = 0;
      if(rele_controller.level){
        if(shared.v>rele_controller.threshold){
          shared.rv = -rele_controller.cmd;
          // shared.rw = -rele_controller.cmd;
          rele_controller.level = 0;
        }else{
          shared.rv = rele_controller.cmd;
          // shared.rw = rele_controller.cmd;
        }
      }else{
        if(shared.v<-rele_controller.threshold){
          shared.rv = rele_controller.cmd;
          // shared.rw = rele_controller.cmd;
          rele_controller.level = 1;
        }else{
          shared.rv = -rele_controller.cmd;
          // shared.rw = -rele_controller.cmd;
        }
      }
      // get accel sensor
      char Buffer[100];
		  kh4_measure_acc((char *)Buffer, dsPic);
      sensor_data.ax = ((short)(Buffer[0] | Buffer[1]<<8)>>4)/1000.0;
      // kh4_measure_gyro((char *)Buffer, dsPic);
      // sensor_data.wz = ((short)((Buffer[40] | Buffer[41]<<8)))*KH4_GYRO_DEG_S; // convert to [deg/s]
      
      pthread_mutex_lock(&shared.mutex);
      shared.vel_l = shared.rv;
      shared.vel_r = shared.rv;
      pthread_mutex_unlock(&shared.mutex); 
      /* RECORD DATA:  last_time shared.v, shared.rv, shared.w, shared.rw*/
      // printf("V: %.3f-%.3f W: %.3f-%.3f \n", shared.v, shared.rv, shared.w, shared.rw);
      if (log_file != NULL) {
        // fprintf(log_file, "%ld,%.3f,%.3f,%.3f,%.3f\n", 
        //   now, shared.v, shared.rv, shared.w, shared.rw);
        fprintf(log_file, "%ld,%.3f,%.3f, %.3f\n", 
          now, shared.v, shared.rv*100.0f/KH4_MAX_OPENLOOP_SPEED_PWM_100, sensor_data.ax);
      }
    }
    // usleep(50000);  // Actualización cada 50ms (20Hz)
    usleep(20000);  // Actualización cada 20ms (50Hz)
  }
  // Limpieza antes de salir
  printf("\nApagando sistema...\n");
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy

  close(newsockfd);
  close(sockfd);
  kh4_set_speed(0, 0, dsPic);
  kh4_SetMode(kh4RegIdle, dsPic);
  pthread_mutex_destroy(&shared.mutex);
    
  printf("Sistema apagado correctamente.\n");
  return 0;
}




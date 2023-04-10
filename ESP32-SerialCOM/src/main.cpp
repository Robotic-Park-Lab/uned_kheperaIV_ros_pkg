#include <Arduino.h>
#include "WiFi.h"

uint32_t chipId = 0;
char ssid[] = "ROBOTICPARK_LAB_N";
IPAddress local_IP(192,168,0,41);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,0,0);

// char pass[] = "";
int status = WL_IDLE_STATUS;     // the WiFi radio's status
int count = 0;

void setup(){
    Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
    delay(1000);
    if(WiFi.config(local_IP, gateway, subnet)) {
      Serial.println("Static IP Configured");
    }
    else {
      Serial.println("Static IP Configuration Failed");
    }
    // WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid);
    // WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}

void loop() {
  delay(1000);
  count += 1;
  Serial.println(count);
}
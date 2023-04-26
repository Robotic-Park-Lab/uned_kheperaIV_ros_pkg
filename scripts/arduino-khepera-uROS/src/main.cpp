#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFiNINA.h>

int cont=0;

char ssid[] = "MOVISTAR_D647";
char password[] = "97A1C7B2468B5B85EEC2";
IPAddress ip(192, 168, 0, 15);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 0, 0);
int status = WL_IDLE_STATUS;     // the Wifi radio's status

int i2c_address = 0x04;

void setup(){
  //iniciamos el puerto serie
  Serial.begin(9600);
  while (!Serial);
  Serial.println("INIT ARDUINO!");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);

  }

  // WiFi.config(ip, gateway, subnet);

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to open SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid);
    // WiFi.config(ip, gateway, subnet);
    // wait 10 seconds for connection:
    delay(1000);
  }
  Serial.println("Red Wi-Fi conectada. ");
  
}

void loop(){
  //Imprimimos el valor del contador
  Serial.print("Contador: ");
  Serial.println(cont);
  
  //incrementamos el contador y esperamos un segundo
  cont++;
  delay(1000);
}
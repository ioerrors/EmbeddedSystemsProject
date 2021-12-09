#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
unsigned long startTime;
unsigned long timeElapsed;

SoftwareSerial softSerial(D6, D7); //RX, TX

const short CLEAR_WAIT = 10;
const short SERIAL_WAIT = 10;
//Related to wifi initialization
const char *WIFI_SSID = "SensorHub";
const short PORT = 8080;
IPAddress selfip(192, 168, 0, 69);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
WiFiServer server(PORT);
 
void setup() 
{
  delay(1000);
  Serial.begin(9600);
  softSerial.begin(9600); 
  
  //configure wifi server
  WiFi.softAPConfig(selfip, gateway, subnet);
  WiFi.softAP(WIFI_SSID);
  //configure wifi client
  server.begin();
}
 
void loop() 
{
  WiFiClient client = server.available();
  if(client)
  {
    while(client.connected())
    {
      if(softSerial.available())
      {
        delay(SERIAL_WAIT);
        byte packetSize = softSerial.available();
        byte* packet = (byte*) malloc(packetSize);
        softSerial.read(packet, packetSize);
        client.write(packetSize);
        for(int I = 0; I < packetSize; I++)
        {
          Serial.print(packet[I]);
          Serial.print(" ");
        }
        Serial.println();
        for(int I = 0; I < packetSize; I++)
        {
          client.write(packet[I]);
        }
        free(packet);
        packet = NULL;
        client.flush();
      }
      if(client.available())
      {
        while(client.available() > 0)
        {
          byte packetSize = client.read();
          while(client.available() != packetSize);
          byte* packet = (byte*) malloc(packetSize);
          for(int I = 0; I < packetSize; I++)
          {
            packet[I] = client.read();
          }
          softSerial.write(packet, packetSize);
        }
      }
    }
  }
}

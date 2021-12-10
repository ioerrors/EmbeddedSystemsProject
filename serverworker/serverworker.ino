#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
unsigned long startTime;
unsigned long timeElapsed;

SoftwareSerial softSerial(D6, D7); //RX, TX

const short PACKET_WAIT_MS = 5000;
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
    Serial.println("Have client");
    while(client && client.connected() && !server.available())
    {
      if(softSerial.available())
      {
        Serial.println("available");
        delay(SERIAL_WAIT);
        byte packetSize = softSerial.read();
        Serial.println(packetSize);
        while(packetSize > softSerial.available());
        byte* packet = (byte*) malloc(packetSize);
        for(int I = 0; I < packetSize; I++)
        {
          packet[I] = softSerial.read();
        }
        client.write(packetSize);
        for(int I = 0; I < packetSize; I++)
        {
          client.write(packet[I]);
          Serial.print(packet[I]);
          Serial.print(" ");
        }
        Serial.println();
        free(packet);
        packet = NULL;
        client.flush();
      }
      if(client.available())
      {
        while(client.available() > 0)
        {
          unsigned long startTime = millis();
          byte packetSize = client.read();
          Serial.println(packetSize);
          while(client.available() != packetSize && millis()-startTime <= PACKET_WAIT_MS);
          if(millis()-startTime <= PACKET_WAIT_MS)
          {
            byte* packet = (byte*) malloc(packetSize);
            for(int I = 0; I < packetSize; I++)
            {
              packet[I] = client.read();
              Serial.print(packet[I]);
              Serial.print(" ");
            }
            Serial.println();
            softSerial.write(packet, packetSize);
          }
          else
          {
            Serial.println("no packet received");
          }
        }
      }
    }
    Serial.println("Lost client");
    client.stop();
  }
}

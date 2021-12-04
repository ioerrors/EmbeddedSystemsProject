#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const byte FETCH_DATA_COMMAND = 17;
const byte PACKET_SIZE = 3*sizeof(float) + sizeof(short) + sizeof(byte);
const short RESPONSE_TIME_MS = 5000;
const char *ssid = "Alexahome";
const char *password = "loranthus";
const short TIMEOUT_MS = 2000; 
unsigned long startTime;
unsigned long timeElapsed;
ESP8266WebServer server(80);
 
void handleSentVar() {
  Serial.println("handleSentVar function called...");
  Serial.write(FETCH_DATA_COMMAND);
  startTime = millis();
  timeElapsed = 0;
  while(Serial.available() < PACKET_SIZE && timeElapsed < TIMEOUT_MS)
  {
    timeElapsed = millis()-startTime;
  }
  if(timeElapsed < TIMEOUT_MS)
  {
    byte* packet = (byte*) malloc(PACKET_SIZE);
    Serial.readBytes(packet, PACKET_SIZE);
    byte boolVal = packet[PACKET_SIZE-1];
    Serial.println(boolVal);
    free(packet);
    server.send(200, "application/json", "{\"received\": true}");
  }
  else
  {
    server.send(500, "application/json", "Serial disconnected");
  }
}
 
void setup() 
{
  delay(1000);
  Serial.begin(9600);
  Serial.println();
  Serial.print("Configuring access point...");
 
  WiFi.softAP(ssid, password);
 
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/data", HTTP_GET, handleSentVar);
  server.begin();
  Serial.println("HTTP server started");
 
}
 
void loop() {
  server.handleClient();
}

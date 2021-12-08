#include <ESP8266WiFi.h>

const byte MAX_PACKET_SIZE = 6;
const short CLIENT_WAIT = 100;
const char *WIFI_SSID = "SensorHub";
const char *HOST_IP = "192.168.0.69";
const short PORT = 8080;
bool connectionFailedPrinted = false;
const short WAIT = 1000;
unsigned long startTime;
WiFiClient client;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setSync(true);
  startTime = millis();
}

void connectToServer()
{
  Serial.println("Connecting to server");
  while(!client.connect(HOST_IP, PORT))
  { 
    Serial.print(".");
    delay(500); 
  }
  Serial.println();
  Serial.println("Connected to server");
}
void loop() {
  // put your main code here, to run repeatedly
  if(client.connected())
  {
    if(client.available() > 0)
    {
      byte messageSize = client.read();
      Serial.println(messageSize);
      byte* message = readMessage(messageSize);
      processSensorData(message, messageSize);
      free(message);
    }
    if(Serial.available())
    {
      String command = Serial.readString();
      byte commandByte = processCommand(command);
    }
  }
  else
  {
    connectToServer();
  }
}
/*
 * Supported Commands
 * PIR_DETECTION_ON
 * PIR_DETECTION_OFF
 * DOOR_DETECTION_ON
 * DOOR_DETECTION_OFF
 * TAMPER_DETECTION_ON
 * TAMPER_DETECTION_OFF
 * PERIODIC_REPORTS_ON
 * PERIODIC_REPORT_OFF
 * SET_HERTZ
 * REQUEST_DATA
 * REQUEST_CONFIG
 * RESET_TAMPER_FLAG
 * RECALIBRATE
 * HELP
 */
byte processCommand(String command)
{
  switch(command)
  {
    case "HELP":
      printCommands();
      break;
    case "PIR_DETECTION_ON":
      byte changeVal = 68;
      break;
    default:
      Serial.println("Unrecognized Command");
  }
}
byte* readMessage(byte messageSize)
{
  while(client.available() != messageSize);
  byte *message = (byte *)malloc(messageSize);
  for(int I = 0; I < messageSize; I++)
  {
    message[I] = client.read();
  }
  return message;
}
void processSensorData(byte* message, byte messageSize)
{
  for(int I = 0; I < messageSize; I++)
  {
    Serial.print(message[I]);
    Serial.print(" ");
  }
  Serial.println();
}

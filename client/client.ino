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
  printCommands();
}


void printCommands() { 
  Serial.println("Supported Commands:");
  Serial.println("PIR_DETECTION_ON");
  Serial.println("PIR_DETECTION_OFF");
  Serial.println("DOOR_DETECTION_ON");
  Serial.println("DOOR_DETECTION_OFF");
  Serial.println("TAMPER_DETECTION_ON");
  Serial.println("TAMPER_DETECTION_OFF");
  Serial.println("PERIODIC_REPORTS_ON");
  Serial.println("PERIODIC_REPORT_OFF");
  Serial.println("SET_HERTZ");
  Serial.println("REQUEST_DATA");
  Serial.println("REQUEST_CONFIG");
  Serial.println("RESET_TAMPER_FLAG");
  Serial.println("RECALIBRATE");
  Serial.println("HELP");
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
      byte change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case "PIR_DETECTION_OFF":
      byte changeVal = 64;
      byte change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case "DOOR_DETECTION_ON":
      byte changeVal = 34;
      byte change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case "DOOR_DETECTION_OFF":
      byte changeVal = 32;
      byte change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case "TAMPER_DETECTION_ON":
      byte changeVal = 17;
      byte change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case "TAMPER_DETECTION_OFF":
      byte changeVal = 16;
      byte change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case "PERIODIC_REPORTS_ON":
      byte changeVal = 136;
      byte change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case "PERIODIC_REPORT_OFF":
      byte changeVal = 128;
      byte change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case "SET_HERTZ":
      byte hertz = 2;
      client.write(hertz);
      System.out.println("Input interval in ms:")
      client.write(changeVal);
      break;
    case "REQUEST_DATA":
      break;
    case "REQUEST_CONFIG":
      break;
    case "RESET_TAMPER_FLAG":
      break;
    case "RECALIBRATE":
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

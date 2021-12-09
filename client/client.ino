#include <ESP8266WiFi.h>

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
enum COMMAND{
  HELP = 0,
  PIR_DETECTION_ON = 1,
  PIR_DETECTION_OFF = 2,
  DOOR_DETECTION_ON = 3,
  DOOR_DETECTION_OFF = 4,
  TAMPER_DETECTION_ON = 5,
  TAMPER_DETECTION_OFF = 6,
  PERIODIC_REPORT_ON = 7,
  PERIODIC_REPORT_OFF = 8,
  SET_HERTZ = 9,
  REQUEST_DATA = 10,
  REQUEST_CONFIG = 11,
  RESET_TAMPER_FLAG = 12,
  RECALIBRATE = 13
};
const static struct {
    COMMAND val;
    const char *str;
}conversion[] = {
    {HELP, "HELP"},
    {PIR_DETECTION_ON, "PIR_DETECTION_ON"},
    {PIR_DETECTION_OFF, "PIR_DETECTION_OFF"},
    {DOOR_DETECTION_ON, "DOOR_DETECTION_ON"},
    {DOOR_DETECTION_OFF, "DOOR_DETECTION_OFF"},
    {TAMPER_DETECTION_ON, "TAMPER_DETECTION_ON"},
    {TAMPER_DETECTION_OFF, "TAMPER_DETECTION_OFF"},
    {PERIODIC_REPORT_ON, "PERIODIC_REPORT_ON"},
    {PERIODIC_REPORT_OFF, "PERIODIC_REPORT_OFF"},
    {SET_HERTZ, "SET_HERTZ"},
    {REQUEST_DATA, "REQUEST_DATA"},
    {REQUEST_CONFIG, "REQUEST_CONFIG"},
    {RESET_TAMPER_FLAG, "RESET_TAMPER_FLAG"},
    {RECALIBRATE, "RECALIBRATE"}
};
COMMAND strToCommand(String str)
{
  char str_array[str.length()];
  str.toCharArray(str_array, str.length());
  char* command = strtok(str_array, " ");
  return charArrToCommand(command);
}
COMMAND charArrToCommand (const char* str)
{
   int j;
   for (j = 0;  j < sizeof (conversion) / sizeof (conversion[0]);  ++j)
       if (!strcmp (str, conversion[j].str))
           return conversion[j].val;    
   return conversion[0].val;
}
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
byte processCommand(String commandStr)
{
  byte* packet = NULL;
  byte packetSize = 0;
  byte changeVal = 0;
  byte change = 0;
  COMMAND command = strToCommand(commandStr);
  switch(command)
  {
    case HELP:
      printCommands();
      break;
    case PIR_DETECTION_ON:
      changeVal = 68;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case PIR_DETECTION_OFF:
      changeVal = 64;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case DOOR_DETECTION_ON:
      changeVal = 34;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case DOOR_DETECTION_OFF:
      changeVal = 32;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case TAMPER_DETECTION_ON:
      changeVal = 17;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case TAMPER_DETECTION_OFF:
      changeVal = 16;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case PERIODIC_REPORT_ON:
      changeVal = 136;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case PERIODIC_REPORT_OFF:
      changeVal = 128;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case SET_HERTZ:
      byte hertz = 2;
      client.write(hertz);
      Serial.println("Input interval in ms:");
      while(!Serial.available());
      String intervalString = Serial.readString();
      char intervalCharArr[intervalString.length()];
      intervalString.toCharArray(intervalCharArr, intervalString.length());
      int interval = atoi(intervalCharArr);
      client.write(interval);
      break;
    case REQUEST_DATA:

      break;
    case REQUEST_CONFIG:
      break;
    case RESET_TAMPER_FLAG:
      break;
    case RECALIBRATE:
      break;
    default:
      Serial.println("Unrecognized Command");
  }
  if(packet != NULL)
  {
    for(int I = 0; I < packetSize; I++)
    {
      client.write(changeVal);
    }
    free(packet);
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

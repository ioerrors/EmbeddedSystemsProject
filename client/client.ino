#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
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
enum COMMAND {
  HELP,
  PIR_DETECTION_ON,
  PIR_DETECTION_OFF,
  DOOR_DETECTION_ON,
  DOOR_DETECTION_OFF,
  TAMPER_DETECTION_ON,
  TAMPER_DETECTION_OFF,
  PERIODIC_DETECTION_ON,
  PERIODIC_DETECTION_OFF,
  SET_HERTZ,
  REQUEST_DATA,
  REQUEST_CONFIG,
  RESET_TAMPER_FLAG,
  RECALIBRATE
};

const static struct {
    COMMAND      val;
    const char *str;
}conversion[] = {
    {HELP, "HELP"},
    {PIR_DETECTION_ON, "PIR_DETECTION_ON"},
    {PIR_DETECTION_OFF, "PIR_DETECTION_OFF"},
    {DOOR_DETECTION_ON, "DOOR_DETECTION_ON"},
    {DOOR_DETECTION_OFF, "DOOR_DETECTION_OFF"},
    {TAMPER_DETECTION_ON, "TAMPER_DETECTION_ON"},
    {TAMPER_DETECTION_OFF, "TAMPER_DETECTION_OFF"},
    {PERIODIC_DETECTION_ON, "PERIODIC_DETECTION_ON"},
    {PERIODIC_DETECTION_OFF, "PERIODIC_DETECTION_OFF"},
    {SET_HERTZ, "SET_HERTZ"},
    {REQUEST_DATA, "REQUEST_DATA"},
    {REQUEST_CONFIG, "REQUEST_CONFIG"},
    {RESET_TAMPER_FLAG, "RESET_TAMPER_FLAG"},
    {RECALIBRATE, "RECALIBRATE"}
};

COMMAND  str2enum (const char* str)
{
   int j;
   for (j = 0;  j < sizeof (conversion) / sizeof (conversion[0]);  ++j)
       if (!strcmp (str, conversion[j].str))
           return conversion[j].val;    
   return conversion[0].val;
}

typedef union {
  unsigned short shortVal;
  byte bytes[sizeof(unsigned short)];
} unsignedShortBytes;

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
byte processCommand(String command)
{
  const char* thing = command;
  COMMAND thisOne = str2enum(thing)
  byte packetSize = 0;
  byte changeVal = 0;
  byte change = 0;
  byte* message = 0;
  switch(thisOne)
  {
    case HELP:
      printCommands();
      break;
    case PIR_DETECTION_ON:
      changeVal = 68;
      change = 1;
      packetSize = 2;
      client.write(packetSize);
      client.write(change);
      client.write(changeVal);
      break;
    case PIR_DETECTION_OFF:
      packetSize = 2;
      client.write(packetSize);
      changeVal = 64;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case DOOR_DETECTION_ON:
      packetSize = 2;
      client.write(packetSize);
      changeVal = 34;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case DOOR_DETECTION_OFF:
      packetSize = 2;
      client.write(packetSize);
      changeVal = 32;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case TAMPER_DETECTION_ON:
      packetSize = 2;
      client.write(packetSize);
      changeVal = 17;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case TAMPER_DETECTION_OFF:
      packetSize = 2;
      client.write(packetSize);
      changeVal = 16;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case PERIODIC_REPORTS_ON:
      packetSize = 2;
      client.write(packetSize);
      changeVal = 136;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case PERIODIC_REPORT_OFF:
      packetSize = 2;
      client.write(packetSize);
      changeVal = 128;
      change = 1;
      client.write(change);
      client.write(changeVal);
      break;
    case SET_HERTZ:
      packetSize = 3;
      client.write(packetSize);
      byte hertz = 2;
      client.write(hertz);
      Serial.println("Input interval in ms:")
      while(!Serial.available());
      unsignedShortBytes interval.shortVal = (unsigned short)atoi(Serial.readString());
      for (int i = 0; i < 2; i++) {
        client.write(interval.bytes[i])
      }
      break;
    case REQUEST_DATA:
      packetSize = 1;
      client.write(packetSize);
      change = 4;
      client.write(change);
      while(!client.available);
      byte messageSize = client.read();
      Serial.println(messageSize);
      message = readMessage(messageSize);
      processSensorData(message, messageSize);
      free(message);
      break;
    case REQUEST_CONFIG:
      packetSize = 1;
      client.write(packetSize);
      change = 8;
      client.write(change);
      while(!client.available);
      byte messageSize = client.read();
      Serial.println(messageSize);
      message = readMessage(messageSize);
      processConfigData(message, messageSize);
      free(message);
      break;
    case RESET_TAMPER_FLAG:
      packetSize = 1;
      client.write(packetSize);
      change = 16;
      client.write(change);
      break;
    case RECALIBRATE:
      packetSize = 1;
      client.write(packetSize);
      change = 32;
      client.write(change);
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
  Serial.print("Sensor data ");
  boolean ultra = true;
  if(message[0] < 128) {
    ultra = false;
  }
  message[0]<<3;
  if(message[0] < 128) {
    Serial.println("On Demand Report:");
  } else {
    Serial.println("Periodic Report:");
  }
  if (!ultra) {
    Serial.println("Ultrasonic sensor off.");
  }
  message[0]<<1;
  if(message[0] < 128) {
    Serial.println("Tampering not detected");
  } else {
    Serial.println("Tampering detected!");
  }
  message[0]<<1;
  if(message[0] < 128) {
    Serial.println("Door closed.");
  } else {
    Serial.println("Door open.");
  }
  message[0]<<1;
  if(message[0] < 128) {
    Serial.println("No motion detected.");
  } else {
    Serial.println("Motion detected.");
  }
  //starting after lead byte
  for(int I = 1; I < messageSize; I++)
  {
    if (message[0] >= 128 && I == 2) {
      Serial.print("Ultrasonic Distance");
      Serial.print(message[I]);
      Serial.print(" cm");
    } else {
      Serial.print(message[I]);
    }
  }
  Serial.println();
}

void processConfigData(byte* message) {
  Serial.println("Config data:");
  if (message[0] % 2 == 0) {
    Serial.println("Tamper Detection set to off.");
  } else {
    Serial.println("Tamper Detection set to on.");
  }
  message[0]>>1;
  if (message[0] % 2 == 0) {
    Serial.println("Door sensor set to off.");
  } else {
    Serial.println("Door sensor set to on.");
  }
  message[0]>>1;
  if (message[0] % 2 == 0) {
    Serial.println("PIR sensor set to off.");
  } else {
    Serial.println("PIR sensor set to on.");
  }
  message[0]>>1;
  if (message[0] % 2 == 0) {
    Serial.println("Periodic reports set to off.");
  } else {
    Serial.println("Periodic reports set to on.");
  }  
}

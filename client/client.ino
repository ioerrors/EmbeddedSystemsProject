#include <ESP8266WiFi.h>
#include <string.h>
#include <stdio.h>
/*
 * Supported Commands
 * PIR_DETECTION_ON *
 * PIR_DETECTION_OFF *
 * DOOR_DETECTION_ON *
 * DOOR_DETECTION_OFF *
 * TAMPER_DETECTION_ON *
 * TAMPER_DETECTION_OFF *
 * PERIODIC_REPORTS_ON
 * PERIODIC_REPORT_OFF
 * SET_HERTZ
 * REQUEST_DATA
 * REQUEST_CONFIG *
 * RESET_TAMPER_FLAG *
 * RECALIBRATE *
 * HELP *
 */
typedef union {
  unsigned short shortVal;
  byte bytes[sizeof(unsigned short)];
} unsignedShortBytes;
typedef union{
  unsigned long longVal;
  byte bytes[sizeof(unsigned long)];
}unsignedLongBytes;
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

const byte TAMPER_BYTE_VALUE = 1;
const byte DOOR_BYTE_VALUE = 2;
const byte PIR_BYTE_VALUE = 4;
const byte PERIODIC_BYTE_VALUE=8;

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
   {
       if (!strcmp (str, conversion[j].str))
       {
           return conversion[j].val;
       }
   }    
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
unsigned long commandCount;
WiFiClient client;
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID);
 
  while (WiFi.status() != WL_CONNECTED) 
  {
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


void printCommands() 
{ 
  Serial.println("Supported Commands:");
  Serial.println("PIR_DETECTION_ON");
  Serial.println("PIR_DETECTION_OFF");
  Serial.println("DOOR_DETECTION_ON");
  Serial.println("DOOR_DETECTION_OFF");
  Serial.println("TAMPER_DETECTION_ON");
  Serial.println("TAMPER_DETECTION_OFF");
  Serial.println("PERIODIC_REPORT_ON");
  Serial.println("PERIODIC_REPORT_OFF");
  Serial.println("SET_HERTZ");
  Serial.println("REQUEST_DATA");
  Serial.println("REQUEST_CONFIG");
  Serial.println("RESET_TAMPER_FLAG");
  Serial.println("RECALIBRATE");
  Serial.println("HELP");
  Serial.println();
  Serial.println("Turn off periodic reporting before changing the hertz value");
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
      byte* message = readMessage(messageSize);
      processSensorData(message, messageSize);
      free(message);
    }
    if(Serial.available())
    {
      commandCount++;
      String command = Serial.readString();
      byte commandByte = processCommand(command);
      Serial.print("There have been ");
      Serial.print(commandCount);
      Serial.println(" commands sent");
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
  byte* returnMessage;
  byte returnMessageSize;
  COMMAND command = strToCommand(commandStr);
  switch(command)
  {
    case PIR_DETECTION_ON:
      packetSize = 3;
      packet = (byte*) malloc(packetSize);
      packet[0] = 2;
      packet[1] = 1;
      packet[2] = 68;
      break;
    case PIR_DETECTION_OFF:
      packetSize = 3;
      packet = (byte*) malloc(packetSize);
      packet[0] = 2;
      packet[1] = 1;
      packet[2] = 64;
      break;
    case DOOR_DETECTION_ON:
      packetSize = 3;
      packet = (byte*) malloc(packetSize);
      packet[0] = 2;
      packet[1] = 1;
      packet[2] = 34;
      break;
    case DOOR_DETECTION_OFF:
      packetSize = 3;
      packet = (byte*) malloc(packetSize);
      packet[0] = 2;
      packet[1] = 1;
      packet[2] = 32;
      break;
    case TAMPER_DETECTION_ON:
      packetSize = 3;
      packet = (byte*) malloc(packetSize);
      packet[0] = 2;
      packet[1] = 1;
      packet[2] = 17;
      break;
    case TAMPER_DETECTION_OFF:
      packetSize = 3;
      packet = (byte*) malloc(packetSize);
      packet[0] = 2;
      packet[1] = 1;
      packet[2] = 16;
      break;
    case PERIODIC_REPORT_ON:
      packetSize = 3;
      packet = (byte*) malloc(packetSize);
      packet[0] = 2;
      packet[1] = 1;
      packet[2] =136;
      break;
    case PERIODIC_REPORT_OFF:
      packetSize = 3;
      packet = (byte*) malloc(packetSize);
      packet[0] = 2;
      packet[1] = 1;
      packet[2] = 128;
      break;
    case SET_HERTZ:
    {
      Serial.println("Input interval in ms:");
      while(!Serial.available());
      unsignedShortBytes interval;
      String input = Serial.readString();
      char thing_array[input.length()];
      input.toCharArray(thing_array, input.length());
      interval.shortVal = (unsigned short)atoi(thing_array);
      Serial.println(interval.shortVal);
      packetSize = 4;
      packet = (byte*) malloc(packetSize);
      packet[0] = 3;
      packet[1] = 2;
      packet[2] = interval.bytes[0];
      packet[3] = interval.bytes[1];
      break;
    }
    case REQUEST_DATA:
      packetSize = 2;
      packet = (byte*) malloc(packetSize);
      packet[0] = 1;
      packet[1] = 4;
      for(int I = 0; I < packetSize; I++)
      {
        client.write(packet[I]);
      }
      while(!client.available());
      returnMessageSize = client.read();
      returnMessage = readMessage(returnMessageSize);
      processSensorData(returnMessage, returnMessageSize);
      free(packet);
      free(returnMessage);
      packet = NULL;
      returnMessage = NULL;
      break;
    case REQUEST_CONFIG:
      packetSize = 2;
      packet = (byte*) malloc(packetSize);
      packet[0] = 1;
      packet[1] = 8;
      for(int I = 0; I < packetSize; I++)
      {
        client.write(packet[I]);
      }
      while(!client.available());
      returnMessageSize = client.read();
      returnMessage = readMessage(returnMessageSize);
      processConfigData(returnMessage[0]);
      free(packet);
      free(returnMessage);
      packet = NULL;
      returnMessage = NULL;
      break;
    case RESET_TAMPER_FLAG:
      packetSize = 2;
      packet = (byte*) malloc(packetSize);
      packet[0] = 1;
      packet[1] = 16;
      break;
    case RECALIBRATE:
      packetSize = 2;
      packet = (byte*) malloc(packetSize);
      packet[0] = 1;
      packet[1] = 32;
      break;
    default:
      printCommands();
      break;
  }
  if(packet != NULL)
  {
    for(int I = 0; I < packetSize; I++)
    {
      client.write(packet[I]);
    }
    free(packet);
  }
  return 0;
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
const byte IS_DISTANCE_INCLUDED_BIT = 128;
const byte LAST_SHORT_BIT = 1;
const byte PIR_SENSOR_BIT = 2;
const byte DOOR_SENSOR_BIT  = 4;
const byte TAMPER_SENSOR_BIT = 8;
const byte PERIODIC_BIT = 16;
const byte COUNTER_BITS = 96;
void processSensorData(byte* message, byte messageSize)
{
  if(messageSize > 0)
  {
    byte headerByte = message[0];
    Serial.print("This message is  ");
    if((headerByte & PERIODIC_BIT) != PERIODIC_BIT)
    {
      Serial.print("not ");
    }
    Serial.println("a periodic message");
    if((headerByte & PIR_SENSOR_BIT) == PIR_SENSOR_BIT)
    {
      Serial.println("The PIR sensor has detected motion");
    }
    else
    {
      Serial.println("No reading has been detected from the PIR sensor");
    }
    if((headerByte & DOOR_SENSOR_BIT) == DOOR_SENSOR_BIT)
    {
      Serial.println("The door sensor has been opened");
    }
    else
    {
      Serial.println("No reading has been detected from the door sensor");
    }
    if((headerByte & TAMPER_SENSOR_BIT) == TAMPER_SENSOR_BIT)
    {
      Serial.println("The device has been tampered with");
    }
    else
    {
      Serial.println("No reading has been detected from the tamper sensor");
    }
    //Calculate Distance
    byte lastShortBit = 0;
    bool distanceIncluded = ((headerByte & IS_DISTANCE_INCLUDED_BIT) == IS_DISTANCE_INCLUDED_BIT);
    if(distanceIncluded)
    {
      lastShortBit = (headerByte & LAST_SHORT_BIT);
      unsignedShortBytes distance;
      distance.bytes[1] = lastShortBit;
      distance.bytes[0] = message[1];
      Serial.print("Ultrasonic distance (cm): ");
      Serial.println(distance.shortVal);
    }
    else
    {
      Serial.println("Ultrasonic distance is not included");
    }
    unsignedLongBytes messageCounter;
    messageCounter.longVal = 0;
    byte offset = 1 + (int) distanceIncluded;
    byte counterByteCount = (headerByte & COUNTER_BITS) >> 5;
    for(int I = 0; I < counterByteCount; I++)
    {
      messageCounter.bytes[I] = message[(offset + counterByteCount) - 1 - I];
    }
    Serial.print("This is message number ");
    Serial.print(messageCounter.longVal);
    Serial.print(" from the sensor hub");
  }
  else
  {
    Serial.println("Packet was malformed");
  }
  Serial.println();
  Serial.println();
}
void processConfigData(byte message) {
  Serial.println("Config data:");
  if ((message & TAMPER_BYTE_VALUE) == TAMPER_BYTE_VALUE) {
    Serial.println("Tamper Detection set to on.");
  } else {
    Serial.println("Tamper Detection set to off.");
  }
  if ((message & DOOR_BYTE_VALUE) == DOOR_BYTE_VALUE) {
    Serial.println("Door sensor set to on.");
  } else {
    Serial.println("Door sensor set to off.");
  }
  if ((message & PIR_BYTE_VALUE) == PIR_BYTE_VALUE) {
    Serial.println("PIR sensor set to on.");
  } else {
    Serial.println("PIR sensor set to off.");
  }
  if ((message & PERIODIC_BYTE_VALUE) == PERIODIC_BYTE_VALUE) {
    Serial.println("Periodic reports set to on.");
  } else {
    Serial.println("Periodic reports set to off.");
  }  
}

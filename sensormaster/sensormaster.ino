
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
/*
  ASSUMPTIONS ABOUT VALUE SIZES
  short: 2 bytes
  float: 4 bytes
  byte: 1 byte
*/
//Typedef to access bits of a short
typedef union {
  unsigned short shortVal;
  byte bytes[sizeof(unsigned short)];
} unsignedShortBytes;
//typedef to access bits of an integer
typedef union{
  unsigned long longVal;
  byte bytes[sizeof(unsigned long)];
}unsignedLongBytes;

//Accelerometer bounds struct
struct AccelerometerBounds {
  float xMin;
  float xMax;
  float yMin;
  float yMax;
  float zMin;
  float zMax;
};

//Periodic data variables
const short PERIODIC_UPPER_LIMIT = 10000;
const short PERIODIC_LOWER_LIMIT = 100;

//const command values
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
 */
const byte CHANGE_VALUE = 1;
const byte SET_HERTZ = 2;
const byte REQUEST_DATA = 4;
const byte CONFIG_BYTE_REQUEST = 8;
const byte RESET_TAMPER_FLAG = 16;
const byte RECALIBRATE = 32;

//Bytes related to the new configuration byte request
const byte CHANGE_TAMPER_VALUE = 16;
const byte CHANGE_DOOR_VALUE = 32;
const byte CHANGE_PIR_VALUE = 64;
const byte CHANGE_PERIODIC_VALUE = 128;

//Bytes consts relating to configuration byte
const byte TAMPER_BYTE_VALUE = 1;
const byte DOOR_BYTE_VALUE = 2;
const byte PIR_BYTE_VALUE = 4;
const byte PERIODIC_BYTE_VALUE=8;

//Pin definitions
const byte PIR_PIN = 7;
const byte TRIG_PIN = 6;
const byte ECHO_PIN = 5;
const byte DOOR_PIN = 4;

//other consts
const byte MESSAGE_COUNT_OFFSET = 32;
const byte MESSAGE_COUNTER_OFFSET = 1;
const byte DISTANCE_THRESHOLD_CM = 511;

unsignedShortBytes periodicLength; // 1 Hz

//Accelerometer values
const short ACCELEROMETER_IDENTIFIER = 54321;
const float accelOffset = 0.4;
const Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(ACCELEROMETER_IDENTIFIER);
AccelerometerBounds accelBounds;

byte configurationByte;
//flags for sensors
boolean stateMagDoor; // 0  close / 1 open
boolean statePIRSensor;
boolean handlingRequest;
bool tampered;

byte packetSize;

unsigned long startTime;

unsignedLongBytes messageCount;

void setup() 
{
  //Initialize serial busses
  while (!Serial);
  Serial.begin(9600);
  while (!Serial2);
  Serial2.begin(9600);
  //Initialize accelerometer
  if (!accel.begin()) 
  {
    Serial.println("Oops, no LSM303 detected ... Check your wiring!");
  }
  accel.setRange(LSM303_RANGE_4G);
  accel.setMode(LSM303_MODE_NORMAL);

  calibrateAccelerometer();
  //Set pin modes
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  //Initialize sensor states
  tampered = false;
  statePIRSensor = false;
  stateMagDoor = false;

  //Set start time
  startTime = millis();
  configurationByte = 7;
  packetSize = 0;
  messageCount.longVal = 0;
  handlingRequest=false;
  periodicLength.shortVal = 1000;
}
//!! I like this
void calibrateAccelerometer() 
{
  tampered = false;
  sensors_vec_t accelerometerData = getAccelerometerData();
  accelBounds.xMin = accelerometerData.x - accelOffset;
  accelBounds.xMax = accelerometerData.x + accelOffset;
  accelBounds.yMin = accelerometerData.y - accelOffset;
  accelBounds.yMax = accelerometerData.y + accelOffset;
  accelBounds.zMin = accelerometerData.z - accelOffset;
  accelBounds.zMax = accelerometerData.z + accelOffset;
}

bool checkAccelerometer() 
{
  sensors_vec_t accelData = getAccelerometerData();
  bool movedXPos = (accelBounds.xMax < accelData.x);
  bool movedXNeg = (accelBounds.xMin > accelData.x);
  bool movedYPos = (accelBounds.yMax < accelData.y);
  bool movedYNeg = (accelBounds.yMin > accelData.y);
  bool movedZPos = (accelBounds.zMax < accelData.z);
  bool movedZNeg = (accelBounds.zMin > accelData.z);
  return (movedXPos || movedXNeg
          || movedYPos || movedYNeg
          || movedZPos || movedZNeg);
}

void loop() {
  // --------------------------------------------
  // PERIODIC REPORT:
  // & system's behavior meaningfully changes
  // according to the collected sensor data
  if (millis() - startTime > periodicLength.shortVal &&
      (configurationByte & PERIODIC_BYTE_VALUE) == PERIODIC_BYTE_VALUE) {
    readSensorData();
    messageCount.longVal += MESSAGE_COUNTER_OFFSET;
    byte* packet = createPacket(true);
    Serial2.write(packetSize);
    for(int I = 0; I < packetSize; I++)
    {
      Serial.print(packet[I]);
      Serial.print(" ");
      Serial2.write(packet[I]);
    }
    free(packet);
    packet = NULL;
    startTime = millis();
  } 
  if(Serial2.available()) 
  {
    delay(10);
    packetSize = Serial2.available();
    byte* packet = malloc(packetSize);
    Serial2.readBytes(packet, packetSize);
    byte commandByte = packet[0];
    if(commandByte == CHANGE_VALUE)
    {
      Serial.println("Changing values");
      byte configRequestByte = packet[1];
      processNewConfigByte(configRequestByte);
    }
    else if(commandByte == SET_HERTZ)
    {
      Serial.println("Setting hertz");
      unsignedShortBytes newValue;
      newValue.shortVal = 0;
      Serial.println(Serial2.available());
      for(int I = 0; I < sizeof(unsigned short); I++)
      {
        newValue.bytes[I] = packet[I+1];
      }
      Serial.println(newValue.shortVal);
      if(newValue.shortVal >= PERIODIC_LOWER_LIMIT && newValue.shortVal <= PERIODIC_UPPER_LIMIT)
      {
        periodicLength.shortVal = newValue.shortVal;
        Serial.print("New periodic value: ");
        Serial.println(periodicLength.shortVal);
      }
      else
      {
        Serial.println("Requested value is out of range");
      }
    }
    else if(commandByte == REQUEST_DATA)
    {
      Serial.println("requesting on demand data");
      readSensorData();
      messageCount.longVal += MESSAGE_COUNTER_OFFSET;
      byte* packet = createPacket(false);
      Serial2.write(packetSize);
      for(int I = 0; I < packetSize; I++)
      {
        Serial.print(packet[I]);
        Serial.print(" ");
        Serial2.write(packet[I]);
      }
      free(packet);
      packet = NULL;
    }
    else if(commandByte == CONFIG_BYTE_REQUEST)
    {
      Serial.println("Request for config");
      Serial.println(configurationByte);
      Serial2.write((byte) 1);
      Serial2.write(configurationByte);
    }
    else if(commandByte == RESET_TAMPER_FLAG)
    {
      Serial.println("Reset tamper flag");
      tampered = false;
    }
    else if(commandByte == RECALIBRATE)
    {
      Serial.println("Calibrating accelerometer");
      calibrateAccelerometer();
    }
    else
    {
      Serial.print("Command byte ");
      Serial.print(commandByte);
      Serial.println(" did not have a command");
    }
    free(packet);
  }
}

void processNewConfigByte(byte newConfigByte)
{
  byte SHOULD_CHANGE_DOOR = newConfigByte & CHANGE_DOOR_VALUE;
  byte SHOULD_CHANGE_PIR = newConfigByte & CHANGE_PIR_VALUE;
  byte SHOULD_CHANGE_TAMPER = newConfigByte & CHANGE_TAMPER_VALUE;
  byte SHOULD_CHANGE_PERIODIC = newConfigByte & CHANGE_PERIODIC_VALUE;
  if(SHOULD_CHANGE_DOOR == CHANGE_DOOR_VALUE)
  {
    Serial.println("changing Door Value");
    if(((configurationByte & DOOR_BYTE_VALUE) == DOOR_BYTE_VALUE)
      && ((newConfigByte & DOOR_BYTE_VALUE) == 0))
    {
      configurationByte = configurationByte & (255-DOOR_BYTE_VALUE);
    }
    else
    {
      configurationByte = configurationByte | (newConfigByte & DOOR_BYTE_VALUE);
    }
  }
  if(SHOULD_CHANGE_PIR == CHANGE_PIR_VALUE)
  {
    Serial.println("changing PIR Value");
    if(((configurationByte & PIR_BYTE_VALUE) == PIR_BYTE_VALUE)
      && ((newConfigByte & PIR_BYTE_VALUE) == 0))
    {
      configurationByte = configurationByte & (255-PIR_BYTE_VALUE);
    }
    else
    {
      configurationByte = configurationByte | (newConfigByte & PIR_BYTE_VALUE);
    }
  }
  if(SHOULD_CHANGE_TAMPER == CHANGE_TAMPER_VALUE)
  {
    Serial.println("changing Tamper Value");
    if(((configurationByte & TAMPER_BYTE_VALUE) == TAMPER_BYTE_VALUE)
      && ((newConfigByte & TAMPER_BYTE_VALUE) == 0))
    {
      configurationByte = configurationByte & (255-TAMPER_BYTE_VALUE);
    }
    else
    {
      configurationByte = configurationByte | (newConfigByte & TAMPER_BYTE_VALUE);
    }
  }
  if(SHOULD_CHANGE_PERIODIC == CHANGE_PERIODIC_VALUE)
  {
    Serial.println("changing Periodic Value");
    if(((configurationByte & PERIODIC_BYTE_VALUE) == PERIODIC_BYTE_VALUE)
      && ((newConfigByte & PERIODIC_BYTE_VALUE) == 0))
    {
      configurationByte = configurationByte & (255-PERIODIC_BYTE_VALUE);
    }
    else
    {
      configurationByte = configurationByte | (newConfigByte & PERIODIC_BYTE_VALUE);
    }
  }
}
void readSensorData() {
  // ------------------------------------------------
  // MEANINGFUL USE OF SENSOR DATA:
  // What is configurationByte?
  // configurationByte is a byte indicating 
  // which sensor between the PIR and Door 
  // turn on other sensor sampling.
  // if select senses no entrance event,
  // returns just select value

  // if select does detect entrance event
  // other sensors are turned on to report to end user
  // ------------------------------------------------
  byte checkTamper = configurationByte & TAMPER_BYTE_VALUE;
  byte checkPIR = configurationByte & PIR_BYTE_VALUE;
  byte checkDoor = configurationByte & DOOR_BYTE_VALUE;
  if (checkTamper == TAMPER_BYTE_VALUE) {
    if(!tampered)
    {
      tampered = checkAccelerometer();
    }
  }
  else
  {
    tampered = false;
  }
  if (checkPIR == PIR_BYTE_VALUE)
  {
    statePIRSensor = digitalRead(PIR_PIN);
  }
  else
  {
    statePIRSensor = false;
  }
  if (checkDoor == DOOR_BYTE_VALUE)
  {
    stateMagDoor = digitalRead(DOOR_PIN);
  }
  else
  {
    stateMagDoor = false;
  }
}
byte* createPacket(boolean periodic)
{
  unsignedShortBytes distance;
  byte leadByte = 0;
  boolean includeDistance = false;
  boolean checkTamper = (configurationByte & TAMPER_BYTE_VALUE) == TAMPER_BYTE_VALUE;
  boolean checkPIR = (configurationByte & PIR_BYTE_VALUE) == PIR_BYTE_VALUE;
  boolean checkDoor = (configurationByte & DOOR_BYTE_VALUE) == DOOR_BYTE_VALUE;
  packetSize = 1;
  leadByte += (statePIRSensor) ? 2 : 0;
  leadByte += (stateMagDoor) ? 4 : 0;
  leadByte += (tampered) ? 8 : 0;
  leadByte += (periodic) ? 16 : 0;
  if (stateMagDoor || statePIRSensor)
  {
    includeDistance = true;
    distance.shortVal = getDistance();
    leadByte += 128;
    packetSize ++;
    if (distance.shortVal >= 256 && distance.shortVal < 512)
    {
      leadByte += 1;
    }
    if(distance.shortVal >= 512)
    {
      distance.shortVal = 0;
    }
  }
  int messageCounterSize = 1;
  for(int I = 0; I < sizeof(unsigned long)-1; I++)
  {
    if(messageCount.bytes[I] == 255)
    {
      messageCount.bytes[I] = 0;
      messageCount.bytes[I+1] ++;
    }
  }
  if(messageCount.longVal >= pow(2,8)-1) //if it is over 2^8, needs another byte
  {
    messageCounterSize ++;
  }
  if(messageCount.longVal >= pow(2,16)-1) //if it is over 2^16, needs another byte
  {
    messageCounterSize ++;
  }
  if(messageCount.longVal >= pow(2,24))//if it is over 2^24, hit the max message count, roll back to 0
  {
    messageCount.longVal = 0;
    messageCounterSize = 1;
  }
  leadByte += MESSAGE_COUNT_OFFSET * messageCounterSize;
  packetSize += messageCounterSize;
  byte* packet = malloc(packetSize);
  packet[0] = leadByte;
  if(includeDistance)
  {
    packet[1] = distance.bytes[0];  
  }
  for(int I = 0; I < messageCounterSize; I++)
  {
    byte messageIndex = messageCounterSize - 1 - I;
    byte packetIndex = I + (int) includeDistance + 1;
    packet[packetIndex] = messageCount.bytes[messageIndex];
  }
  return packet;
}
sensors_vec_t getAccelerometerData()
{
  sensors_event_t event;
  accel.getEvent(&event);
  return event.acceleration;
}

short getDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return (short) pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
}

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
  short shortVal;
  byte bytes[sizeof(short)];
} shortBytes;
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

//const command values
const byte FETCH_DATA_COMMAND = 17;
const byte SELECT_IS_PIR = 1;
const byte SELECT_IS_DOOR = 0;
const byte SELECT_IS_EITHER = 2;
const byte RESET_TAMPER_FLAG = 8;
const byte RECALIBRATE = 9; //should also reset tamper flag

//Bytes consts relating to configuration message
const byte TAMPER_BYTE_VALUE = 1;
const byte DOOR_BYTE_VALUE = 2;
const byte PIR_BYTE_VALUE = 4;

//Pin definitions
const byte PIR_PIN = 7;
const byte TRIG_PIN = 6;
const byte ECHO_PIN = 5;
const byte DOOR_PIN = 4;

//other consts
const byte MESSAGE_COUNT_OFFSET = 32;
const byte DISTANCE_THRESHOLD_CM = 511;

short PERIODIC_LENGTH = 1000; // 1 Hz

//Accelerometer values
const short ACCELEROMETER_IDENTIFIER = 54321;
const float accelOffset = 0.4;
const Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(ACCELEROMETER_IDENTIFIER);
AccelerometerBounds accelBounds;

long unsigned int pause = 1000;

byte configurationByte;
//flags for sensors
boolean stateMagDoor; // 0  close / 1 open
boolean statePIRSensor;
bool tampered;

byte packetSize;

unsigned long startTime;

unsignedLongBytes messageCount;

void setup() {
  //Initialize serial busses
  while (!Serial);
  Serial.begin(9600);
  while (!Serial2);
  Serial2.begin(9600);

  //Initialize accelerometer
  if (!accel.begin()) {
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
}

//!! I like this
void calibrateAccelerometer() {
  tampered = false;
  sensors_vec_t accelerometerData = getAccelerometerData();
  accelBounds.xMin = accelerometerData.x - accelOffset;
  accelBounds.xMax = accelerometerData.x + accelOffset;
  accelBounds.yMin = accelerometerData.y - accelOffset;
  accelBounds.yMax = accelerometerData.y + accelOffset;
  accelBounds.zMin = accelerometerData.z - accelOffset;
  accelBounds.zMax = accelerometerData.z + accelOffset;
}

bool checkAccelerometer() {
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
  if (millis() - startTime > PERIODIC_LENGTH
      && PERIODIC_LENGTH >= 100) {

    // ------------------------------------------------
    // MEANINGFUL USE OF SENSOR DATA:
    // What is select?
    // select is a byte indicating which sensor
    // between the PIR and Door turn on
    // other sensor sampling.
    // if select senses no entrance event,
    // returns just select value

    // if select does detect entrance event
    //  other sensors are turned on to report to end user
    // ------------------------------------------------

    // sets stateMagDoor & statePIRSensor
    if (configurationByte != 0) {
      readSensorData();
      messageCount.longVal ++;
      byte* packet = createPacket(true);
      free(packet);
      packet = NULL;
    }
    //packet = NULL;
    startTime = millis();
  }
  if(Serial2.available()){
    while(Serial2.available()){
      Serial.print(Serial2.read());
      Serial.print(" ");
    }
    Serial.println();
  }
  /*
    if(Serial1.available()){
    //Serial.println("received message");
    // --------------------------------------------
    // command responses:
    // take in command
    int value = Serial1.read();

    // ON DEMAND data collection & packet creation
    if(value == FETCH_DATA_COMMAND)
    {
      readSensorData();
      byte* packet = createPacket(false);
      free(packet);
      packet = NULL;
    }

    // SET PERIODIC REPORT INTERVAL:
    if (value >= 100 && value <= 1000)
    {
      PERIODIC_LENGTH = value;
    }

    // TURN OFF Periodic REPORT:
    if (value < 0)
    {
      PERIODIC_LENGTH = -1; //turn off periodic reports
    }

    // change select sensor
    if(value == SELECT_IS_DOOR
    || value == SELECT_IS_PIR
    || value == SELECT_IS_EITHER)
    {

    }

    // reset tamper flag
    if(value == RESET_TAMPER_FLAG)
    {

    }

    // recalibrate tamper boundaries
    if(value == RECALIBRATE)
    {

    }
    }
  */
}
void readSensorData() {
  byte checkTamper = configurationByte & TAMPER_BYTE_VALUE;
  byte checkPIR = configurationByte & PIR_BYTE_VALUE;
  byte checkDoor = configurationByte & DOOR_BYTE_VALUE;
  if (checkTamper == TAMPER_BYTE_VALUE) {
    tampered = checkAccelerometer();
  }
  if (checkPIR == PIR_BYTE_VALUE)
  {
    statePIRSensor = digitalRead(PIR_PIN);
  }
  if (checkDoor == DOOR_BYTE_VALUE)
  {
    stateMagDoor = digitalRead(DOOR_PIN);
  }
}
byte* createPacket(boolean periodic)
{
  shortBytes distance;
  byte leadByte = 0;
  boolean includeDistance = false;
  boolean checkTamper = (configurationByte & TAMPER_BYTE_VALUE) == TAMPER_BYTE_VALUE;
  boolean checkPIR = (configurationByte & PIR_BYTE_VALUE) == PIR_BYTE_VALUE;
  boolean checkDoor = (configurationByte & DOOR_BYTE_VALUE) == DOOR_BYTE_VALUE;
  packetSize = 1;
  if (checkPIR && statePIRSensor)
  {
    leadByte += 2; //flip three bit
  }
  if (checkDoor && stateMagDoor)
  {
    leadByte += 4; //flip fourth bit
  }
  if (checkTamper && tampered)
  {
    leadByte += 8; //flip second bit
  }
  if(periodic)
  {
    leadByte += 16;  
  }
  if (checkPIR || checkDoor)
  {
    includeDistance = true;
    distance.shortVal = getDistance();
    leadByte += 128;
    packetSize ++;
    if (distance.shortVal >= 256 && distance.shortVal < 512)
    {
      leadByte += 1;
    }
  }
  int messageCounterSize = 1;
  if(messageCount.longVal >= pow(2,8)) //if it is over 2^8, needs another byte
  {
    messageCounterSize ++;
  }
  if(messageCount.longVal >= pow(2,16)) //if it is over 2^16, needs another byte
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

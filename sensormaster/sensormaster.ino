#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
/*
ASSUMPTIONS ABOUT VALUE SIZES
short: 2 bytes
float: 4 bytes
byte: 1 byte
*/
//typedefs for byte conversion
typedef union {
  float floatVal;
  byte bytes[sizeof(float)];
}floatBytes;
typedef union {
  float shortVal;
  byte bytes[sizeof(short)];
}shortBytes;

//const command values
const byte FETCH_DATA_COMMAND = 17;
const byte SELECT_IS_PIR = 1;
const byte SELECT_IS_DOOR = 0;
const byte SELECT_IS_EITHER = 2;
const byte RESET_TAMPER_FLAG = 8;
const byte RECALIBRATE = 9; //should also reset tamper flag


// tamper flag
volatile bool tamper;

// Door or PIR or Either triggers other sensor reports
volatile byte select = 0; //default is select = door


const short PERIODIC_LENGTH = 3000;
const byte PACKET_SIZE = 3*sizeof(float) + sizeof(short) + sizeof(byte);
const byte PIR_PIN = 7;
const byte TRIG_PIN = 6;
const byte ECHO_PIN = 5;
const byte DOOR_PIN = 4;
const byte DISTANCE_THRESHOLD_CM = 200;
const short ACCELEROMETER_IDENTIFIER = 54321;
const short CALIBRATION_TIME_MS = 4000;

//Non-const values
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(ACCELEROMETER_IDENTIFIER);
long unsigned int pause = 5000;
volatile boolean stateMagDoor; // 0  close / 1 open
volatile boolean statePIRSensor;
unsigned long startTime;

void setup() {
  //Initialize serial busses
  while (!Serial);
  Serial.begin(9600);
  while (!Serial1);
  Serial1.begin(9600);
  
  //Initialize accelerometer
  if (!accel.begin()) {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!"); 
  }
  accel.setRange(LSM303_RANGE_4G);
  lsm303_accel_range_t new_range = accel.getRange();
  accel.setMode(LSM303_MODE_NORMAL);
  lsm303_accel_mode_t new_mode = accel.getMode();
  
  //Set pin modes
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);

  tamper = false;

  //Set start time
  startTime = millis();
}


void loop() {
  if(Serial1.available()){

    // --------------------------------------------
    // command responses:

    // take in command
    int value = Serial1.read();

    // ON DEMAND data collection & packet creation
    if(value == FETCH_DATA_COMMAND){
      byte* packet = createPacket();
      Serial1.write(packet, PACKET_SIZE);
      free(packet);
      packet = NULL;
    }

    // change select sensor
    if(value == SELECT_IS_DOOR
    || value == SELECT_IS_PIR
    || value == SELECT_IS_EITHER) {
      select = value;
    }


    // reset tamper flag
    if(value == RESET_TAMPER_FLAG) {
      tamper = false;
    }

    // recalibrate tamper boundaries
    if(value == RECALIBRATE) {
      tamper = false;
      // ! ! ! recalibarte tamper boundaries
    }

    // --------------------------------------------
    // periodic packet creation
    // & system's behavior meaningfully changes 
    // according to the collected sensor data
    else if (millis() - startTime > PERIODIC_LENGTH) {
      // ------------------------------------------------
      // What is select?
      // select is a byte indicating which sensor 
      // between the PIR and Door turn on 
      // other sensor sampling.
      // if select senses no entrance event,
      // returns just select value
      
      // if select does detect entrance event
      //  proceed with if statements acording to meaning
      // ------------------------------------------------

      // sets stateMagDoor // 0  close / 1 open
      // sets statePIRSensor
      byte booleanByte = calculateBooleanByte();

      // if select = door
      if (select == 0) {
          if (stateMagDoor == true) { // door is open
              byte* packet = createPacket();
              Serial1.write(packet, PACKET_SIZE);
              free(packet);
              packet = NULL;
          }
          else { // transmits just PIR and DOOR data
              Serial1.write(booleanByte, sizeof(byte));
          }
      }

      // if select == PIR    
      if (select == 1) {
          if (statePIRSensor == true) { // motion detected
              byte* packet = createPacket();
              Serial1.write(packet, PACKET_SIZE);
              free(packet);
              packet = NULL;
          }
          else { // transmits just PIR and DOOR data
              Serial1.write(booleanByte, sizeof(byte));
          }
      }

      // if select is either
      if (select == 2) {
        if (statePIRSensor == true  // motion detected
            || stateMagDoor == true) { // door open
            byte* packet = createPacket();
            Serial1.write(packet, PACKET_SIZE);
            free(packet);
            packet = NULL;
        }
        else { // transmits just PIR and DOOR data
            Serial1.write(booleanByte, sizeof(byte));
        }
      }
      startTime = millis();
    }

  }
}


byte* createPacket(){
    sensors_event_t event = getAccelerometerData();
    floatBytes accelX;
    floatBytes accelY;
    floatBytes accelZ;
    shortBytes distance;
    accelX.floatVal = event.acceleration.x;
    accelY.floatVal = event.acceleration.y;
    accelZ.floatVal = event.acceleration.z;
    distance.shortVal = getDistance();
    byte booleanByte = calculateBooleanByte();
    byte* packet = malloc(PACKET_SIZE);
    //add accelX to packet
    byte xStart = 0;
    byte yStart = sizeof(float);
    byte zStart = sizeof(float)*2;
    byte distanceStart = sizeof(float)*3;
    byte boolValStart = sizeof(float)*3 + sizeof(short);
    for(int I = xStart; I < yStart; I++){
      packet[I] = accelX.bytes[I];
    }
    //add accelY to packet
    for(byte I = yStart; I < zStart; I++){
      packet[I] = accelY.bytes[I];
    }
    //add accelZ to packet
    for(byte I = zStart; I < distanceStart; I++){
      packet[I] = accelZ.bytes[I];
    }
    //add distance
    for(byte I = distanceStart; I < boolValStart; I++)
    {
      packet[I] = distance.bytes[I];
    }
    packet[boolValStart] = booleanByte;
    return packet;
}
sensors_event_t getAccelerometerData(){
    sensors_event_t event;
    accel.getEvent(&event);
    return event;
}
//calculates the byte value corresponding to the tracked booleans
//Currently, this is the PIR sensor and the door sensor
//from least significant bit to most, the values are order as:
//door sensor, PIR sensor
byte calculateBooleanByte()
{
  noInterrupts();
  stateMagDoor = digitalRead(DOOR_PIN);
  statePIRSensor = digitalRead(PIR_PIN);
  boolean currentDoorValue = stateMagDoor;
  boolean currentPIRValue = statePIRSensor;
  interrupts();
  byte finalValue = 0;
  finalValue += (currentDoorValue) ? 1 : 0;
  finalValue += (currentPIRValue) ? 2 : 0;
  return finalValue;
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

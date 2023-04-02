#include <LoRa.h>

// Define the possible states for each set
enum Recovery_State { IDLE_REC, DETECT_LAUNCH_REC, VALID_FLIGHT_REC, DEPLOY_DROGUE_REC, DEPLOY_MAIN_REC };
enum Ignition_State { IDLE_IGN, DETECT_LAUNCH_IGN, VALID_FLIGHT_IGN, LOCKOUT_IGN, IGNITE_IGN };

// Define the initial state for each set
Recovery_State StateSet1 = IDLE_REC;
Ignition_State StateSet2 = IDLE_IGN;

#include <LoRa.h>

/*
stage label bit (1 bit)
status (1 byte, chars)
GPS data —> 2 32 bit long  (lat & longitude)
altimeter —> int 32 bit
IMU (gyro & accelerometer) —> 6 16 bit integers 
*/

//struct for packet
typedef struct {
  uint16_t xGyro; //4
  uint16_t yGyro; //4
  uint16_t zGyro; //4
  uint16_t xAcc; //4
  uint16_t yAcc; //4
  uint16_t zAcc; //4

} __attribute__((packed)) IMU;

typedef struct {
  char header = 0x55; //1
  char padding0; //1
  char stage; //1
  char padding1; //1
  uint32_t latitude; //4
  char padding2; //1
  uint32_t longitude; //4
  char padding3; //1
  uint32_t altitude; //4
  char padding4; //1
  IMU imuPacket; // 24
} __attribute__((packed)) Packet;

//struct for transmission
const int intended_packet_size = sizeof(Packet);

void setup() {
  //Lora Setup
  Serial.begin(115200);
  while (!Serial);
  if (!LoRa.begin(915E6)) {
    while (1);
  }
}

void send_to_lora(uint8_t * packet) {
  //writing with packet
  LoRa.beginPacket();
  LoRa.write(packet, intended_packet_size);
  LoRa.endPacket();
}

//ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent isses with sideways travel

//ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
float HEIGHT_FLOOR = 0.00776714; //TEMP height floor in 8-miles height floor
float MAIN_TRIGGER = 0.0142045; //TEMP height floor in 8-miles main trigger

void loop() {
  // PLACEHOLDER READ DATA
  //data = read_telemetry();
  int data[] = {0,0,0,0,0,0,0};
  // Update state machine Recovery State
  switch (StateSet1) {
    case IDLE_REC: //State to represent the rocket pre launch 
      //Placeholder condition for detecting launch
      //0 = acceleration index
      if (data[0] >= 0) { //movement of rocket is the current assumption for launch
        StateSet1 = DETECT_LAUNCH_REC;
      }
      break;
    case DETECT_LAUNCH_IGN: //state to represent the rocket post launch (can mistake heavy jostling for launch)
      //Placeholder condition for confirming launch
      //1 = altitude 
      if (data[1] >= HEIGHT_FLOOR) { //floor reached
        StateSet1 = VALID_FLIGHT_REC;
      }
      //detect false launch 
      //2 = altitude_trend
      if (data[2] == 0) {//if not tredning anywhere
        StateSet1 = IDLE_REC;
      }
      break;
    case VALID_FLIGHT_REC: //State to represent further confirmation of launch with heigth floor
      //Placeholder condition for detecting apogee and triggering drouge deploy
      if (data[2] < 0) {//if treding down
        StateSet1 = DEPLOY_DROGUE_REC;
      }
      break;
    case DEPLOY_DROGUE_REC: //State to trigger drouge deploy at apogee
      //deploy_drogue();
      if (data[1] <= MAIN_TRIGGER) {
        StateSet1 = DEPLOY_MAIN_REC; 
      }
      break;
    case DEPLOY_MAIN_REC: //state to trigger main deploy at floor
      //deploy_main();
      //END STATE
      break;
  }

  // Update state machine for Ignition State
  switch (StateSet2) {
    case IDLE_IGN:
      //Placeholder condition for detecting launch
      if (data[0] >= 0){ //movement of rocket is the current assumption for launch
        StateSet2 = DETECT_LAUNCH_IGN;
      }
      break;
    case DETECT_LAUNCH_IGN:
      //Placeholder condition for confirming launch
      if (data[1] >= HEIGHT_FLOOR) { //floor reached
        StateSet2 = VALID_FLIGHT_IGN;
      }
      //detect false launch  (we dropped the rocket)
      if (data[2] == 0) {//if not tredning anywhere
        StateSet2 = IDLE_IGN;
      }
      break;
    case VALID_FLIGHT_IGN:
      //if any tilt exists
      if (abs(data[3]) >= 5) {//
        StateSet2 = LOCKOUT_IGN;
      }
      // detect time to ignite
      if (data[4] < 0) {//if slowing down
        StateSet2 = IGNITE_IGN;
      }
      break;
      StateSet2 = LOCKOUT_IGN;
      break;
    case LOCKOUT_IGN:
      //END STATE
      break;
    case IGNITE_IGN:
      //do_ignite();
      //END STATE
      break;
  }
  broadcast_data();
  // Add any other code you want to run continuously in your loop here
}


void broadcast_data() {
  //set up IMU packet
  IMU imuP;
  imuP.xGyro = 0;
  imuP.yGyro = 0;
  imuP.zGyro = 0;
  imuP.xAcc = 0;
  imuP.yAcc = 0;
  imuP.zAcc = 0;

  //set up transmission Packet
  Packet packet;
  packet.stage = '1';
  packet.latitude = 0;
  packet.longitude = 10;
  packet.altitude = 0;
  packet.imuPacket = imuP;

  //convert to uint8_t packet
  uint8_t * packet_addr = (uint8_t *)(&packet);

  //send to lora function
  send_to_lora(packet_addr);
}
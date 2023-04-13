#include <LoRa.h>

// Define the possible states for each set
enum Recovery_State { IDLE_REC, DETECT_LAUNCH_IGN, VALID_FLIGHT_REC, DEPLOY_MAIN_REC };

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

//initialize packet size based on struct
const int intended_packet_size = sizeof(Packet);

// Define the initial state for each set
Recovery_State StateSet1 = IDLE_REC;

//Setup for state machine
void setup() {
    //Attempt Lora setup
  Serial.begin(115200)
  while (!Serial);
  if (!LoRa.begin(915E6)) {
    while (1);
  }
}

//method to write to LoRa
void send_to_lora(uint8_t * packet) {
  //writing with packet
  LoRa.beginPacket();
  LoRa.write(packet, intended_packet_size);
  LoRa.endPacket();
}

//TODO::ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent issues with sideways travel

//ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
float HEIGHT_FLOOR = 0.00776714; //TEMP height floor in 8-miles
float MAIN_TRIGGER = 0.0142045; //TEMP main trigger in 8-miles

void loop() {
  // PLACEHOLDER READ DATA [TODO::Actually get the data]
  //variable assignment: {acceleration index, altitude, altitude_trend}
  int data[] = {0,0,0,0,0,0,0};

  //TODO::update method --> data = read_telemetry();

  // Update state machine 
  switch (StateSet1) {
    case IDLE_REC: //State to represent the rocket pre launch 
      //Placeholder condition for detecting launch
      //index[0] = acceleration index
      if (data[0] >= 0){ //movement of rocket is the current assumption for launch
        StateSet1 = DETECT_LAUNCH_IGN;
      }
      break;

    //State to represent the rocket post launch (can mistake heavy jostling for launch)
    case DETECT_LAUNCH_IGN: 
      //Placeholder condition for confirming launch
      //index[1] = altitude 
      //Check if height floor reached and set to valid launch
      if (data[1] >= HEIGHT_FLOOR) {
        StateSet1 = VALID_FLIGHT_REC;
      }
      //detect false launch 
      //index[2] = altitude_trend
      if (data[2] == 0) {
        //if altitude is not changing
        StateSet1 = IDLE_REC;
      }
      break;
    
    //State to represent further confirmation of launch with height floor
    case VALID_FLIGHT_REC: 
      //index[2] = atitude trend 
      //check if altitude is trending downwards
      if (data[2] < 0) {
        StateSet1 = DEPLOY_MAIN_REC;
      }
      break;

    //State to trigger main deploy at floor
    case DEPLOY_MAIN_REC: 
      //deploy_main();
      //END STATE
      break;
    
  }
  broadcast_data();
}

void broadcast_data(){
  //set up IMU packet with fake data
  IMU imuP;
  imuP.xGyro = 0;
  imuP.yGyro = 0;
  imuP.zGyro = 0;
  imuP.xAcc = 0;
  imuP.yAcc = 0;
  imuP.zAcc = 0;

  //set up transmission Packet with fake data
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
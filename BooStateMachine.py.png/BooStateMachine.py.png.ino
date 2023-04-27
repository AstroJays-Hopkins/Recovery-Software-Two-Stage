#include <LoRa.h>

// Define the possible states for each set
enum Recovery_State { IDLE_REC = 'I', DETECT_LAUNCH_IGN = 'D', VALID_FLIGHT_REC = 'F', DEPLOY_MAIN_REC = 'M'};

//struct for packet
typedef struct {
  uint16_t thetaGyro; //4
  uint16_t phiGyro; //4
  uint16_t psiGyro; //4
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
  uint32_t altTrend;
  char padding5;  
  IMU imuPacket; // 24
} __attribute__((packed)) Packet;

//initialize packet size based on struct
const int intended_packet_size = sizeof(Packet);

// Define the initial state for each set
Recovery_State StateSet1 = IDLE_REC;

//Setup for state machine
void setup() {
    //Attempt Lora setup
  Serial.begin(115200);
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
  data = pull_test_data();

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
  send_test_data(StateSet1);
  //broadcast_data(); //REMOVED FOR PYTHON TESTING
}

int[] pull_test_data(){
  char inputString[25]; // declare a variable to store the input string
  int i = 0;
  while (Serial.available()) { // check if there is any data available in the serial port
    char c = Serial.read(); // read a character from the serial port
    if (c == '\n') { // check if the character is a newline character
      break; // if yes, exit the loop
    }
    inputString[i] = c; // append the character to the input string
    i++
  }
  inputString[i] = '\0';
 .//TODO: PARSE INPUT AS ARR  
  return formatInputString;
}

void send_test_data(enum Recovery_State state){
  char outputString[25];
  Serial.println();
}

Packet read_telemetry(){
  char raw_data[50];
  int data[3];
  Serial.readBytesUntil("\n", raw_data, 100);
  int data_index = 0;
  uint32_t buffered_value = 0;
  for (int i = 0; i < strlen(raw_data); i++) {//loop through raw chars of serial data input
    if (raw_data[i] == ','){ //when we see a comma reset the number we are counting 
      data[data_index] = buffered_value; //store the summed up value
      data_index += 1; // move on to collecting the next value
      buffered_value = 0; //reset the buffered value we are counting 
    }
    buffered_value = buffered_value*10; //shift left by multiple of 10
    buffered_value += ((int)raw_data[i] - '0'); //store the next char in the string as an int
  }
  IMU imuP;
  imuP.thetaGyro = 0;
  imuP.phiGyro = 0;
  imuP.psiGyro = 0;
  imuP.xAcc = 0;
  imuP.yAcc = 0;
  imuP.zAcc = data[0];

  //set up transmission Packet with fake data
  Packet packet;
  packet.stage = '1';
  packet.latitude = 0;
  packet.longitude = 10;
  packet.altitude = data[1];
  packet.altTrend = data[2];
  packet.imuPacket = imuP;
  return packet;
}

void broadcast_data(){
  //set up IMU packet with fake data
  IMU imuP;
  imuP.thetaGyro = 0;
  imuP.phiGyro = 0;
  imuP.psiGyro = 0;
  imuP.xAcc = 0;
  imuP.yAcc = 0;
  imuP.zAcc = 0;

  //set up transmission Packet with fake data
  Packet packet;
  packet.stage = '1';
  packet.latitude = 0;
  packet.longitude = 10;
  packet.altitude = 0;
  packet.altTrend = 0;
  packet.imuPacket = imuP;

  //convert to uint8_t packet
  uint8_t * packet_addr = (uint8_t *)(&packet);

  //send to lora function
  send_to_lora(packet_addr);
}
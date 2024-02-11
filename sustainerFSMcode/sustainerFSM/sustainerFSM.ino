#include <Wire.h>
#include <LoRa.h>

// Define the possible states for each set
enum Recovery_State { IDLE_REC, DETECT_LAUNCH_REC, VALID_FLIGHT_REC, DEPLOY_DROGUE_REC, DEPLOY_MAIN_REC };
enum Ignition_State { IDLE_IGN, DETECT_LAUNCH_IGN, VALID_FLIGHT_IGN, LOCKOUT_IGN, IGNITE_IGN };

// Define the initial state for each set
Recovery_State StateSet1 = IDLE_REC;
Ignition_State StateSet2 = IDLE_IGN;

//memory
int max_height = 0;

void setup() {
  //Lora Setup
  Serial.begin(115200);
}

void read_telemetry(String result[2]) {
  result[0] = Serial.readStringUntil('\n');
  result[1] = Serial.readStringUntil('\n');
}

int mean(int arr[]){
  int counter = 0;
  for (int i = 0; i <= 10; i++) {
    counter += arr[i];
  } 
  return counter/10;
}

//ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent isses with sideways travel

// ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
float HEIGHT_FLOOR = 100; //0.00776714; // TEMP height floor in 8-miles
float MAIN_TRIGGER = 1000;//0.0142045;  // TEMP main trigger in 8-miles
int APPOGEE_DROP_THRESHOLD = 10;
int IGNITION_SLOW_THRESH = 10;
int data[] = {0, 0, 0, 0, 0, 0, 0};
int alt_trend[] = {0,0,0,0,0,0,0,0,0,0};
int mem_pointer = 0;
int acc_trend[] = {0,0,0,0,0,0,0,0,0,0};


void loop() {
  // PLACEHOLDER READ DATA
  //variable assignment: {acceleration, altitude, altitude_trend, accel_trend, gyro tilt}
  if (data[1] > max_height){
    max_height = data[1];
  }
  String s_data[2];
  read_telemetry(s_data);
  data[0] = s_data[0].toInt(); //accel
  data[1] = s_data[1].toInt(); //alt

  alt_trend[mem_pointer] = data[1];
  acc_trend[mem_pointer] = data[0];
  mem_pointer += 1;
  mem_pointer = mem_pointer % 10;

  data[2] = mean(alt_trend); //alt average
  data[3] = mean(acc_trend); //acc average
  String printout;
  // Update state machine Recovery State
  switch (StateSet1) {

    //State to represent the rocket pre launch 
    case IDLE_REC: 
      printout = "IDLE_REC";
      //Placeholder condition for detecting launch
      //0 = acceleration index
      //check for change in acceleration and set state to detecting launch
      if (data[0] >= 0) {
        StateSet1 = DETECT_LAUNCH_REC;
      }
      break;
    
    //state to represent the rocket post launch (can mistake heavy jostling for launch)
    case DETECT_LAUNCH_IGN:
      printout = "DETECT_LAUNCH_IGN";
      //Placeholder condition for confirming launch
      //1 = altitude 
      if (data[1] >= HEIGHT_FLOOR) { 
        //height floor reached
        StateSet1 = VALID_FLIGHT_REC;
      }
      //detect false launch by looking for altitude trend
      //2 = altitude_trend
      if (data[2] < 0) {
        StateSet1 = IDLE_REC;
      }
      break;

    //State to represent further confirmation of launch with height floor
    case VALID_FLIGHT_REC:
      printout = "VALID_FLIGHT_REC";
      //Placeholder condition for detecting apogee and triggering drouge deploy
      if (data[1] < max_height - APPOGEE_DROP_THRESHOLD) {//if treding down
        StateSet1 = DEPLOY_DROGUE_REC;
      }
      break;
    
    //State to trigger drogue deploy at apogee
    case DEPLOY_DROGUE_REC:
      printout = "DEPLOY_DROGUE_REC";
      //deploy_drogue();
      if (data[1] <= MAIN_TRIGGER) {
        StateSet1 = DEPLOY_MAIN_REC; 
      }
      break;
    
    //State to trigger main deploy at floor
    case DEPLOY_MAIN_REC:
      printout = "DEPLOY_MAIN_REC"; 
      //deploy_main();
      //END STATE
      break;
  }

  // Update state machine for Ignition State
  switch (StateSet2) {
    //rocket idle state
    case IDLE_IGN:
      printout = printout + ", IDLE_IGN";
      //Placeholder condition for detecting launch
      if (data[0] >= 0){ //movement of rocket is the current assumption for launch
        StateSet2 = DETECT_LAUNCH_IGN;
      }
      break;
    //pre-official launch to account for false launches
    case DETECT_LAUNCH_IGN:
      printout = printout + ", DETECT_LAUNCH_IGN";
      //Placeholder condition for confirming launch
      if (data[1] >= HEIGHT_FLOOR) { //floor reached
        StateSet2 = VALID_FLIGHT_IGN;
      }
      //detect false launch  (we dropped the rocket)
      if (data[2] < 0) {//if not tredning anywhere
        StateSet2 = IDLE_IGN;
      }
      break;
    //once valid flight established
    case VALID_FLIGHT_IGN:
      printout = printout + ", VALID_FLIGHT_IGN";
      //if any tilt exists initiate lockout to avoid Missiling
      if (abs(data[4]) >= 5) {//
        StateSet2 = LOCKOUT_IGN;
      }
      // detect time to ignite based on slowing down
      if (data[0] < -IGNITION_SLOW_THRESH) {
        StateSet2 = IGNITE_IGN;
      }
      break;
      StateSet2 = LOCKOUT_IGN;
      break;
    case LOCKOUT_IGN:
      printout = printout + ", LOCKOUT_IGN";
      //END STATE
      break;
    case IGNITE_IGN:
      printout = printout + ", IGNITE_IGN";
      //do_ignite();
      //END STATE
      break;
  }
  Serial.println(printout);
  // broadcast_data();
  // Add any other code you want to run continuously in your loop here
}
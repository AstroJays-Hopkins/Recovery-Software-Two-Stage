// Define the possible states for each set
enum Recovery_State { IDLE_REC, DETECT_LAUNCH_IGN, VALID_FLIGHT_REC, DEPLOY_DROGUE_REC, DEPLOY_MAIN_REC };
enum Ignition_State { IDLE_IGN, DETECT_LAUNCH_IGN, VALID_FLIGHT_IGN, LOCKOUT_IGN, IGNITE_IGN };

// Define the initial state for each set
StateSet1 Recovery_State = IDLE_REC;
StateSet2 Ignition_State = IDLE_IGN;

void setup() {
  // Initialize your hardware and other setup code here
}


//ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent isses with sideways travel

//ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
float HEIGHT_FLOOR = 0.00776714 //TEMP height floor in 8-miles height floor
float MAIN_TRIGGER = 0.0142045 //TEMP height floor in 8-miles main trigger

void loop() {
  // PLACEHOLDER READ DATA
  data = read_telemetry();
  // Update state machine Recovery State
  switch (Recovery_State) {
    case IDLE_REC: //State to represent the rocket pre launch 
      //Placeholder condition for detecting launch
      if (data[acceleration] >= 0){ //movement of rocket is the current assumption for launch
        Recovery_State = DETECT_LAUNCH_IGN;
      }
      break;
    case DETECT_LAUNCH_IGN: //state to represent the rocket post launch (can mistake heavy jostling for launch)
      //Placeholder condition for confirming launch
      if data[altitude] >= HEIGHT_FLOOR{ //floor reached
        Recovery_State = VALID_FLIGHT_REC;
      }
      //detect false launch 
      if (data[altitude_trend] == 0){//if not tredning anywhere
        Recovery_State = IDLE_REC;
      }
      break;
    case VALID_FLIGHT_REC: //State to represent further confirmation of launch with heigth floor
      //Placeholder condition for detecting apogee and triggering drouge deploy
      if (data[altitude_trend] < 0){//if treding down
        Recovery_State = DEPLOY_DROGUE_REC;
      }
      break;
    case DEPLOY_DROGUE_REC: //State to trigger drouge deploy at apogee
      do_drouge();
      if (data[altitude] <= MAIN_TRIGGER){
        Recovery_State = DEPLOY_MAIN_REC; 
      }
      break;
    case DEPLOY_MAIN_REC: //state to trigger main deploy at floor
      do_main();
      //END STATE
      break;
    broadcast_data();
  }

  // Update state machine for Ignition State
  switch (Ignition_State) {
    case IDLE_IGN:
      //Placeholder condition for detecting launch
      if (data[acceleration] >= 0){ //movement of rocket is the current assumption for launch
        Recovery_State = DETECT_LAUNCH_IGN;
      }
      break;
    case DETECT_LAUNCH_IGN:
      //Placeholder condition for confirming launch
      if data[altitude] >= HEIGHT_FLOOR{ //floor reached
        Recovery_State = VALID_FLIGHT_REC;
      }
      //detect false launch 
      if (data[altitude_trend] == 0){//if not tredning anywhere
        Recovery_State = IDLE_REC;
      }
      break;
    case VALID_FLIGHT_IGN:
      //if any tilt exists
      if (abs(data[tilt]) >= 5) {//
        Recovery_State = LOCKOUT_IGN;
      }
      // detect time to ignite
      if (data[acceleration_change] < 0){//if slowing down
        Recovery_State = IGNITE_IGN;
      }
      break;
      Ignition_State = LOCKOUT_IGN;
      break;
    case LOCKOUT_IGN:
      //END STATE
      break;
    case IGNITE_IGN:
      do_ignite();
      //END STATE
      break;
  }
  // Add any other code you want to run continuously in your loop here
}
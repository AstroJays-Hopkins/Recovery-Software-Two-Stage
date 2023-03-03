// Define the possible states for each set
enum Recovery_State { IDLE_REC, DETECT_LAUNCH_IGN, VALID_FLIGHT_REC, DEPLOY_DROGUE_REC, DEPLOY_MAIN_REC };
enum Ignition_State { IDLE_IGN, DETECT_LAUNCH_IGN, VALID_FLIGHT_IGN, LOCKOUT_IGN, IGNITE_IGN };

// Define the initial state for each set
StateSet1 Recovery_State = IDLE_REC;
StateSet2 Ignition_State = IDLE_IGN;

void setup() {
  // Initialize your hardware and other setup code here
}

void loop() {
  // Update state machine for Set 1
  switch (Recovery_State) {
    case IDLE_REC:
      // Do something for state 1 of Set 1
      Recovery_State = DETECT_LAUNCH_IGN;
      break;
    case DETECT_LAUNCH_IGN:
      // Do something for state 2 of Set 1
      Recovery_State = VALID_FLIGHT_REC;
      break;
    case VALID_FLIGHT_REC:
      // Do something for state 3 of Set 1
      Recovery_State = DEPLOY_DROGUE_REC;
      break;
    case DEPLOY_DROGUE_REC:
      // Do something for state 3 of Set 1
      Recovery_State = DEPLOY_MAIN_REC;
      break;
    case DEPLOY_MAIN_REC:
      // Do something for state 3 of Set 1
      Recovery_State = IDLE_REC;
      break;
  }

  // Update state machine for Set 2
  switch (Ignition_State) {
    case IDLE_IGN:
      // Do something for state 1 of Set 2
      Ignition_State = DETECT_LAUNCH_IGN;
      break;
    case DETECT_LAUNCH_IGN:
      // Do something for state 2 of Set 2
      Ignition_State = VALID_FLIGHT_IGN;
      break;
    case VALID_FLIGHT_IGN:
      // Do something for state 3 of Set 2
      Ignition_State = LOCKOUT_IGN;
      break;
    case LOCKOUT_IGN:
      // Do something for state 3 of Set 2
      Ignition_State = IGNITE_IGN;
      break;
    case IGNITE_IGN:
      // Do something for state 3 of Set 2
      Ignition_State = IDLE_IGN;
      break;
  }
  // Add any other code you want to run continuously in your loop here
}
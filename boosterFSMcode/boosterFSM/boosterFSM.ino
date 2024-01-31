#include <LoRa.h>
// Define the possible states for each set
enum Recovery_State
{
  IDLE_REC = 'I',
  DETECT_LAUNCH_IGN = 'D',
  VALID_FLIGHT_REC = 'F',
  DEPLOY_MAIN_REC = 'M'
};


// Define the initial state for each set
Recovery_State StateSet1 = IDLE_REC;

//memory
int max_height = 0;

// Setup for state machine
void setup()
{
  
  // Attempt Lora setup
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

// TODO::ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent issues with sideways travel

// ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
float HEIGHT_FLOOR = 100; //0.00776714; // TEMP height floor in 8-miles
float MAIN_TRIGGER = 1000;//0.0142045;  // TEMP main trigger in 8-miles
int APPOGEE_DROP_THRESHOLD = 10;
int data[] = {0, 0, 0, 0, 0, 0, 0};
int alt_trend[] = {0,0,0,0,0,0,0,0,0,0};
int mem_pointer = 0;
int acc_trend[] = {0,0,0,0,0,0,0,0,0,0};

void loop()
{
  //delay(1000); // Add a delay of 1 second (adjust as needed)
  // PLACEHOLDER READ DATA [TODO::Actually get the data]
  // variable assignment: {acceleration index, altitude, altitude_trend}
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
  // TODO::test

  // Update state machine
  switch (StateSet1)
  {
  case IDLE_REC: // State to represent the rocket pre launch
    Serial.println("IDLE_REC");
    // Placeholder condition for detecting launch
    // index[0] = acceleration index
    if (data[0] >= 0)
    { // movement of rocket is the current assumption for launch
      StateSet1 = DETECT_LAUNCH_IGN;
    }
    break;

  // State to represent the rocket post launch (can mistake heavy jostling for launch)
  case DETECT_LAUNCH_IGN:
    Serial.println("DETECT_LAUNCH_IGN");
    // Serial.println(String(data[1]));
    // Placeholder condition for confirming launch
    // index[1] = altitude
    // Check if height floor reached and set to valid launch
    if (data[1] >= HEIGHT_FLOOR)
    {
      StateSet1 = VALID_FLIGHT_REC;
    }
    // detect false launch
    // index[2] = altitude_trend
    if (data[2] < 0)
    {
      // if altitude is not changing
      StateSet1 = IDLE_REC;
    }
    break;

  // State to represent further confirmation of launch with height floor
  case VALID_FLIGHT_REC:
    // Serial.println("VALID_FLIGHT_REC");
    Serial.println(data[2] - max_height);
    // index[2] = atitude trend
    // check if altitude is trending downwards
    if (data[1] < max_height - APPOGEE_DROP_THRESHOLD)
    {
      //NEEDS TRIGGER WAIT      
      StateSet1 = DEPLOY_MAIN_REC;
    }
    break;

  // State to trigger main deploy at floor
  case DEPLOY_MAIN_REC:
    Serial.println("DEPLOY_MAIN_REC");
    // deploy_main();
    // END STATE
    break;
  }
  // send_test_data(StateSet1);
  // broadcast_data(); //REMOVED FOR PYTHON TESTING
}
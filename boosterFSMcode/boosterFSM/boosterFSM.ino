#include <SPI.h>
#include <SD.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <LoRa.h>
#include "IntersemaBaro.h"


//TODO: check which angle is roll in rocket

// Define the possible states for each set
enum Recovery_State
{
  IDLE_REC = 'I',
  DETECT_LAUNCH_IGN = 'D',
  VALID_FLIGHT_REC = 'F',
  DEPLOY_MAIN_REC = 'M'
};

//%%%%%%%%%%%%%%%%%%%%%%%% FLIGHT OPS PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%//

//TBD
#define FSM_SPEED 250 //manual delay per FSM iteration [helps for testing 0 for launch]
#define LAUNCH_FLOOR 100
#define DECELERATION_DELAY 1000 //delay from when we detect motor burnout before sep
#define BOOSTER_BURN 1000 //milliseconds of expected burn
#define MAIN_DIST_FROM_APPOGEE 10 //-change in alt from apogee before drouge
#define MAIN_PIN 5 //relay pins to be set
#define SEP_PIN 7 


//%%%%%%%%%%%%%%%%%%%%%%%% IMU SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
float ang[3] = {0,0,0};
float ang_basis[3] = {0,0,0};

//%%%%%%%%%%%%%%%%%%%%%%%% ALTIMETER SETUP %%%%%%%%%%%%%%%%%%%%%%%%//

Intersema::BaroPressure_MS5607B baro(true);

// Altitude variables
float avg_alt;
float alt0;
float altitude;
float old_Alt;
float new_Alt;
unsigned long T0;
unsigned long T1;
unsigned long T;

//%%%%%%%%%%%%%%%%%%%%%%%% DATALOGGER SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
int chipSelect = 4;

//%%%%%%%%%%%%%%%%%%%%%%%% FSM SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
// Define the initial state for each set
Recovery_State StateSet1 = IDLE_REC;
int calibrate_length = 500; //sets how many iterations of average sensor readings we take on launch pad
bool seperated = false;

//memory
int max_height = 0;
int base_height = 0;
unsigned long startTime;
bool booster_on = false;
// TODO::ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent issues with sideways travel
// ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
float HEIGHT_FLOOR = 20; //0.00776714; // TEMP height floor in 8-miles
float MAIN_TRIGGER = 1000;//0.0142045;  // TEMP main trigger in 8-miles
int APPOGEE_DROP_THRESHOLD = 10;
float data[] = {0, 0, 0, 0, 0, 0, 0}; //accel, alt, alt average, 
float alt_trend[] = {0,0,0,0,0,0,0,0,0,0};
int mem_pointer = 0;
float acc_trend[] = {0,0,0,0,0,0,0,0,0,0};

//%%%%%%%%%%%%%%%%%%%%%%%% PIN SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
//pin settings
int buzzerPin = 9;

//sensor settings
const int Mode = 1; //1:sensor 2: test from laptop


void read_telemetry_test(String result[2]) {
  result[0] = Serial.readStringUntil('\n');
  result[1] = Serial.readStringUntil('\n');
}

float calculateAngleDifference(float final_angle, float initial_angle) {
  // Calculate the smallest difference between two angles considering wrap-around
  float difference = final_angle - initial_angle;
  if (difference > 180) {
    difference -= 360;
  } else if (difference < -180) {
    difference += 360;
  }
  return difference;
}

float calculateInclination(double pitchDegrees, double yawDegrees) {
    // Convert degrees to radians
    double pitch = pitchDegrees * DEG_TO_RAD;
    double yaw = yawDegrees * DEG_TO_RAD;

    // Calculate direction vector components
    double v_x = cos(pitch) * cos(yaw);
    double v_y = cos(pitch) * sin(yaw);
    double v_z = sin(pitch);

    // Calculate the magnitude of the vector
    double v_magnitude = sqrt(v_x * v_x + v_y * v_y + v_z * v_z);

    // Calculate the inclination angle
    double alpha = acos(sqrt(v_y*v_y + v_z*v_z) / v_magnitude);

    // Convert radians to degrees for the final result
    float alphaDegrees = abs(90 - alpha * RAD_TO_DEG);

    return alphaDegrees;
}

void read_telemetry(float result[3]) {
  //Lin-Accel
  data[0] = 0;
  //altitude
  {
    data[1] = baro.getHeightCentiMeters()/30.48 - alt0;
    Serial.print("Displayed altitude: ");
    Serial.println(data[1]);
    Serial.print("Read altitude: ");
    Serial.println(data[1] + alt0);
    Serial.print("Zero altitude: ");
    Serial.println(alt0);
  }
  //gyro
  {
    sensors_event_t event;
    bno.getEvent(&event);
    ang[1] = calculateAngleDifference(ang_basis[0],(int)event.orientation.x); //write orientation angles to array
    ang[2] = calculateAngleDifference(ang_basis[1],(int)event.orientation.y);
    ang[3] = calculateAngleDifference(ang_basis[2],(int)event.orientation.z);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    Serial.print("ang0: ");
    Serial.println(ang[1]);
    Serial.print("ang1: ");
    Serial.println(ang[2]);
    Serial.print("ang2: ");
    Serial.println(ang[3]);
    // data[4] = (ang[2]*ang[2])+(ang[3]*ang[3]);
    data[4] = calculateInclination(ang[2],ang[3]);
    Serial.print("bad_tilt: ");
    Serial.println(data[4]);
  }
}

void do_sep(){
  delay(DECELERATION_DELAY);
  digitalWrite(SEP_PIN, HIGH);
  seperated = true;
  Serial.println(" SEPARATION !!!!!!!!!!!!!!!!!!!!!!!!!");
}

void do_main(){
  if (!seperated){
    digitalWrite(SEP_PIN, HIGH);
    delay(500);
  }
  digitalWrite(MAIN_PIN, HIGH);
}

void start_flight(){
  //Setup Burnout Code
  startTime = millis();
  booster_on = true;
}

void end_flight(){
  //Setup Burnout Code
  startTime = millis();
  booster_on = false;
}

int calibrate(){
  int song[7] = {440,494,523,587,659,698,784};
  {//Setup Burnout Code
    startTime = millis();
  }
  {//Setup Gyro True North
    for (int i = 0; i<calibrate_length;i++){
      ///Gyro Calibration Summation
      {
        sensors_event_t event;
        bno.getEvent(&event);
        ang_basis[0] += (int)event.orientation.x; //write orientation angles to array
        ang_basis[1] += (int)event.orientation.y;
        ang_basis[2] += (int)event.orientation.z;
      }
    }
    
    //take average
    ang_basis[0]/=calibrate_length;
    ang_basis[1]/=calibrate_length;
    ang_basis[2]/=calibrate_length;

    //correct average
    // Correct average angles
    for (int i = 0; i < 3; i++) {
        if (ang_basis[i] > 180.0) {
            ang_basis[i] -= 360.0;
        } else if (ang_basis[i] < -180.0) {
            ang_basis[i] += 360.0;
        }
    }
  }
  //setup altimeter
  {
    //set starting altitude
    alt0 = 0;
    altitude = 0;
    avg_alt = 0;
    int num_points = 150;
    for (int i=0; i<num_points; i++)
      {
        alt0 += baro.getHeightCentiMeters()/30.48;
        delay(15);
      }
    alt0 /= num_points;            //normalize to the number of samples collected
  }
  tone(buzzerPin, 440); // A4
  delay(50);

  tone(buzzerPin, 494); // B4
  delay(50);

  tone(buzzerPin, 523); // C4
  delay(50);

  tone(buzzerPin, 587); // D4
  delay(50);

  tone(buzzerPin, 659); // E4
  delay(50);

  tone(buzzerPin, 698); // F4
  delay(50);

  tone(buzzerPin, 784); // G4
  delay(100);

  noTone(buzzerPin);
  return 1;
}

float mean(float arr[]){
  int counter = 0;
  for (int i = 0; i <= 10; i++) {
    counter += arr[i];
  } 
  return counter/10;
}


// Setup for state machine
void setup()
{
  // delay(1500);
  //Buzzer Code
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 587); // Turn the buzzer on
  delay(100); // Wait for 500 milliseconds
  tone(buzzerPin, 342); // Turn the buzzer off
  delay(500); 
  noTone(buzzerPin);
  // Attempt Lora setup
  Serial.begin(115200);

  tone(buzzerPin, 587); // Turn the buzzer on
  delay(100); // Wait for 500 milliseconds
  tone(buzzerPin, 342); // Turn the buzzer off
  delay(500); 
  noTone(buzzerPin);  


  //IMU
  bno.begin();
  bno.setExtCrystalUse(false);
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println("6");
  // Serial.println("Sensors Initialized");

  //Altimiter 
  baro.init();

  //SD Card
  //initialize datalogging
  
  if (!SD.begin(chipSelect)) {
    Serial.println("1");
    // Serial.println("Card failed, or not present");
  }
  else{
    File dataFile = SD.open("Flight.txt", FILE_WRITE);
    if (dataFile){
        dataFile.println("2");
        // dataFile.println("Beginning New Flight");
        dataFile.close();
        Serial.println("3");
        // Serial.println("New Datalog Created");
      }
    else{
        Serial.println("4");
        // Serial.println("error opening file Flight.txt");
        return;
      }
    Serial.println("5");
    // Serial.println("Initialization Complete");
  }
  
  //fail case
  if (calibrate() == 0){
    while(1) {
      //failed setup
    }
  }
}


void loop()
{
  delay(FSM_SPEED); // Add a delay of 1 second (adjust as needed)
  // variable assignment: {acceleration, altitude, altitude_mean, accel_mean, abs_gyro_tilt}
  if (data[1] > max_height){
    max_height = data[1];
  }
  if (Mode == 0){
    String s_data[2];
    read_telemetry_test(s_data);
    data[0] = s_data[0].toInt(); //accel
    data[1] = s_data[1].toInt(); //alt
  } else {
    read_telemetry(data);
  }

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
    if (data[0] >= 0 && data[1] > 5)
    { // movement of rocket is the current assumption for launch
      StateSet1 = DETECT_LAUNCH_IGN;
      start_flight();
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
      end_flight();
    }
    break;

  // State to represent further confirmation of launch with height floor
  case VALID_FLIGHT_REC:
    Serial.println("VALID_FLIGHT_REC");
    //Serial.println(data[2] - max_height);
    // index[2] = atitude trend
    // check if altitude is trending downwards
    if (data[1] < max_height - APPOGEE_DROP_THRESHOLD)
    {
      //NEEDS TRIGGER WAIT      
      StateSet1 = DEPLOY_MAIN_REC;
    }
    unsigned long currentTime = millis();
    if (booster_on && currentTime - startTime > BOOSTER_BURN) {
      do_sep();
    }
    break;

  // State to trigger main deploy at floor
  case DEPLOY_MAIN_REC:
    Serial.println("DEPLOY_MAIN_REC");
    do_main();
    // END STATE
    break;
  }
  //Datalogging:
  String data_string = "";

  for (int i = 0;i < 7;i++){
      data_string += data[i];
      if (i < 10) {
        data_string += " ";
      }
  }
  File dataFile = SD.open("Flight.txt", FILE_WRITE);

  if (dataFile){
    dataFile.println(data_string);
    dataFile.close();
  }
  else{
    Serial.println("error opening file Flight.txt");
  }
}
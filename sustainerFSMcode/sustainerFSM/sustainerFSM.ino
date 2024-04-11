#include <SPI.h>
#include <SD.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <LoRa.h>
#include "IntersemaBaro.h"

// Define the possible states for each set
enum Recovery_State { IDLE_REC, DETECT_LAUNCH_REC, VALID_FLIGHT_REC, DEPLOY_DROGUE_REC, DEPLOY_MAIN_REC };
enum Ignition_State { IDLE_IGN, DETECT_LAUNCH_IGN, VALID_FLIGHT_IGN, LOCKOUT_IGN, IGNITE_IGN };


//%%%%%%%%%%%%%%%%%%%%%%%% ACCELEROMETER SETUP %%%%%%%%%%%%%%%%%%%%%%%%//

const int LAsampleSize = 10;
int LARawMin = 0;
int LARawMax = 1023;
float LAX0 = 0;
float LAY0 = 0;
float LAZ0 = 0;

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
int chipSelect = 53;

//%%%%%%%%%%%%%%%%%%%%%%%% FSM SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
// Define the initial state for each set
Recovery_State StateSet1 = IDLE_REC;
Ignition_State StateSet2 = IDLE_IGN;
int calibrate_length = 100; //sets how many iterations of average sensor readings we take on launch pad

//memory
int max_height = 0;
int base_height = 0;
// TODO::ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent issues with sideways travel
// ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
float HEIGHT_FLOOR = 20; //0.00776714; // TEMP height floor in 8-miles
float MAIN_TRIGGER = 30;//0.0142045;  // TEMP main trigger in 8-miles
int APPOGEE_DROP_THRESHOLD = 10;
int IGNITION_SLOW_THRESH = 0.15;
float data[] = {0, 0, 0, 0, 0, 0, 0}; //accel, alt, alt average, 
float alt_trend[] = {0,0,0,0,0,0,0,0,0,0};
int mem_pointer = 0;
float acc_trend[] = {0,0,0,0,0,0,0,0,0,0};
//ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent isses with sideways travel


//%%%%%%%%%%%%%%%%%%%%%%%% PIN SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
//pin settings
int buzzerPin = 4;
const int xInput = A0;
const int yInput = A1;
const int zInput = A2;

//sensor settings
const int Mode = 1; //1:sensor 2: test from laptop


void read_telemetry_test(String result[2]) {
  result[0] = Serial.readStringUntil('\n');
  result[1] = Serial.readStringUntil('\n');
}

// Take samples and return the average
int ReadAxis(int axisPin)
{
	long reading = 0;
	analogRead(axisPin);
	delay(1);
	for (int i = 0; i < LAsampleSize; i++)
	{
	reading += analogRead(axisPin);
	}
	return reading/LAsampleSize;
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

void read_telemetry(float result[3]) {
  //Lin-Accel
  {
    //Read raw values
    int xRaw = ReadAxis(xInput);
    int yRaw = ReadAxis(yInput);
    int zRaw = ReadAxis(zInput);

    // Convert raw values to 'milli-Gs"
    long xScaled = map(xRaw, LARawMin, LARawMax, -3000, 3000);
    long yScaled = map(yRaw, LARawMin, LARawMax, -3000, 3000);
    long zScaled = map(zRaw, LARawMin, LARawMax, -3000, 3000);
    
    // // re-scale to fractional Gs
    float xAccel = xScaled / 1000.0 - LAX0;
    float yAccel = yScaled / 1000.0 - LAY0;
    float zAccel = zScaled / 1000.0 - LAZ0;
    data[0] = zAccel;
    Serial.print("zAccel: ");
    Serial.println(data[0]);
  }
  //altitude
  {
    data[1] = baro.getHeightCentiMeters()/30.48 - alt0;
    Serial.print("Displayed altitude: ");
    Serial.println(data[1]);
    // Serial.print("Read altitude: ");
    // Serial.println(data[1] + alt0);
    // Serial.print("Zero altitude: ");
    // Serial.println(alt0);
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
    data[4] = (ang[2]*ang[2])+(ang[3]*ang[3]);
    Serial.print("bad_tilt: ");
    Serial.println(data[4]);
  }
}

int calibrate(){

  int song[7] = {440,494,523,587,659,698,784};
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
    int num_points = 50;
    for (int i=0; i<num_points; i++)
      {
        alt0 += baro.getHeightCentiMeters()/30.48;
        delay(15);
      }
    alt0 /= num_points;            //normalize to the number of samples collected
  }
  //Setup Accelerometer  
  {
    for (int x = 0; x < calibrate_length; x++){
      //Read raw values
      int xRaw = ReadAxis(xInput);
      int yRaw = ReadAxis(yInput);
      int zRaw = ReadAxis(zInput);

      // Convert raw values to 'milli-Gs"
      long xScaled = map(xRaw, LARawMin, LARawMax, -3000, 3000);
      long yScaled = map(yRaw, LARawMin, LARawMax, -3000, 3000);
      long zScaled = map(zRaw, LARawMin, LARawMax, -3000, 3000);
      
      // // re-scale to fractional Gs
      LAX0 += xScaled / 1000.0;
      LAY0 += yScaled / 1000.0;
      LAZ0 += zScaled / 1000.0;
    }
    LAX0 /= calibrate_length;
    LAY0 /= calibrate_length;
    LAZ0 /= calibrate_length;
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

int mean(int arr[]){
  int counter = 0;
  for (int i = 0; i <= 10; i++) {
    counter += arr[i];
  } 
  return counter/10;
}


// Setup for state machine
void setup()
{
  
  // Attempt Lora setup
  Serial.begin(115200);

  //Buzzer Code
  pinMode(buzzerPin, OUTPUT);

  //LA setup
  // analogReference(EXTERNAL);
  //TODO Fix Analoge Reference

  
  //IMU
  bno.begin();
  bno.setExtCrystalUse(true);
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println("Sensors Initialized");

  //Altimiter 
  baro.init();

  //SD Card
  //initialize datalogging
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  else{
    File dataFile = SD.open("Flight.txt", FILE_WRITE);

    if (dataFile){
        dataFile.println("Beginning New Flight");
        dataFile.close();
        Serial.println("New Datalog Created");
      }
    else{
        Serial.println("error opening file Flight.txt");
        return;
      }
    Serial.println("Initialization Complete");
  }
  
  //fail case
  if (calibrate() == 0){
    while(1) {
      //failed setup
    }
  }
}



void loop() {
  delay(50); // Add a delay of 1 second (adjust as needed)
  //variable assignment: {acceleration, altitude, altitude_trend, accel_trend, gyro tilt}
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
    case DETECT_LAUNCH_REC:
      printout = "DETECT_LAUNCH_REC";
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
      if (abs(data[4]) >= 400) {//
        StateSet2 = LOCKOUT_IGN;
      }
      // detect time to ignite based on slowing down
      else if (data[0] < -IGNITION_SLOW_THRESH) {
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
  Serial.println(printout);
  // broadcast_data();
  // Add any other code you want to run continuously in your loop here
}
#include <Adafruit_BNO055.h>
#include <LoRa.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// uncomment to enable debug printing, must define BEFORE debug.h
// #define DEBUG
#include "IntersemaBaro.h"
#include "debug.h"

// Define the possible states for each set
enum class RecoveryState {
    IDLE_REC,
    DETECT_LAUNCH_REC,
    VALID_FLIGHT_REC,
    DEPLOY_DROGUE_REC,
    DEPLOY_MAIN_REC
};
enum class IgnitionState {
    IDLE_IGN,
    DETECT_LAUNCH_IGN,
    VALID_FLIGHT_IGN,
    LOCKOUT_IGN,
    IGNITE_IGN
};

struct TelemetryEvent {
    float altFt;
    float altAvgFt;
    float linAccel;
    float linAccelAvg;
    float theta_x;
    float theta_y;
    float theta_z;
    float inclination;
};

union StatusFlags {
    struct {
        uint8_t camera_on : 1;
        uint8_t igniter_fired : 1;
        uint8_t drogue_deployed : 1;
        uint8_t main_deployed : 1;
        uint8_t reserved : 4;
    } bit;
    uint8_t reg;
};

//%%%%%%%%%%%%%%%%%%%%%%%% FLIGHT OPS PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%//

// TODO:
//  1) start timer at alt = 5
//  2) implement camera code - sus only

// TBD
#define FSM_SPEED 0     // delay per FSM iteration [helps for testing 0 for launch] (ms).
#define ARM_ALT 5       // altitude to arm statemachines at (ft).
#define LAUNCH_FLOOR 10 // altitude to transition into ascent mode (ft).
#define MAIN_CHUTE_ALT 50         // altitude (rel ground) to trigger main chute (ft).
#define BOOSTER_BURN 1000         // expected booster burn time (ms).
#define DECELERATION_DELAY 500    // delay from motor burnout to separation (ms).
#define IGNITION_DELAY 500        // delay from separation to ignition (ms).
#define APPOGEE_DROP_THRESHOLD 10 // -change in alt from apogee before drouge
#define LOCKOUT_DEGREE 20         // degrees of bad tilt squared before lockout
#define ALT_CAL_SAMPLES 20        // samples to take to calibrate 0-point.
#define AVG_CNT 4 // number of sensor samples to average to smooth noise. MUST BE <= 10.

#define MAIN_PIN 11 // relay pins to be set
#define DROUGE_PIN 10
#define SUSTAINER_PIN 12
#define CAMERA_PIN 14

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
float ang_basis[3] = {0, 0, 0};

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
RecoveryState recoveryState = RecoveryState::IDLE_REC;
IgnitionState ignitionState = IgnitionState::IDLE_IGN;
int calibrate_length =
    100; // sets how many iterations of average sensor readings we take on launch pad

// memory
int max_height = 0;
int base_height = 0;
unsigned long startTime;
bool booster_on = false;
// TODO::ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to
// prevent issues with sideways travel ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
// float data[] = {0, 0, 0, 0, 0, 0, 0}; // accel, alt, alt average,
int mem_pointer = 0;
float alt_trend[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float acc_trend[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to prevent
// isses with sideways travel

//%%%%%%%%%%%%%%%%%%%%%%%% PIN SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
// pin settings
int buzzerPin = 4;
const int xInput = A1;
const int yInput = A2;
const int zInput = A3;

// sensor settings
const int Mode = 1; // 1:sensor 2: test from laptop

StatusFlags statusFlags;
TelemetryEvent currentData;
File logFile;

void read_telemetry_test(String result[2]) {
    result[0] = Serial.readStringUntil('\n');
    result[1] = Serial.readStringUntil('\n');
}

// Take samples and return the average
int ReadAxis(int axisPin) {
    long reading = 0;
    analogRead(axisPin);
    delay(1);
    for (int i = 0; i < LAsampleSize; i++) {
        reading += analogRead(axisPin);
    }
    return reading / LAsampleSize;
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
    double alpha = acos(sqrt(v_y * v_y + v_z * v_z) / v_magnitude);

    // Convert radians to degrees for the final result
    float alphaDegrees = abs(90 - alpha * RAD_TO_DEG);

    return alphaDegrees;
}

void read_telemetry() {
    // Lin-Accel
    {
        // Read raw values
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
        currentData.linAccel = yAccel;
        DEBUG_PRINT_LABELED("yAccel", currentData.linAccel);
    }
    // altitude
    {
        currentData.altFt = baro.getHeightCentiMeters() / 30.48 - alt0;
        DEBUG_PRINT_LABELED("Displayed altitude:", currentData.altFt);
    }
    // gyro
    {
        sensors_event_t event;
        bno.getEvent(&event);
        currentData.theta_x = calculateAngleDifference(
            ang_basis[0], (int)event.orientation.x); // write orientation angles to array
        currentData.theta_y =
            calculateAngleDifference(ang_basis[1], (int)event.orientation.y);
        currentData.theta_z =
            calculateAngleDifference(ang_basis[2], (int)event.orientation.z);
        delay(BNO055_SAMPLERATE_DELAY_MS);

        // data[4] = (ang[2]*ang[2])+(ang[3]*ang[3]);
        currentData.inclination =
            calculateInclination(currentData.theta_y, currentData.theta_z);
        DEBUG_PRINT_LABELED("ang0", currentData.theta_x);
        DEBUG_PRINT_LABELED("ang1", currentData.theta_y);
        DEBUG_PRINT_LABELED("ang2", currentData.theta_z);
        DEBUG_PRINT_LABELED("bad_tilt", currentData.inclination);
    }
}

void playTones(uint8_t piezoPin, int* notes, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        tone(piezoPin, notes[i]);
        delay(150);
    }
    noTone(piezoPin);
}

int calibrate() {
    int song[7] = {440, 494, 523, 587, 659, 698, 784}; // {A,B,C,D,E,F,G}4
    {                                                  // Setup Burnout Code
        startTime = millis();
    }
    { // Setup Gyro True North
        for (int i = 0; i < calibrate_length; i++) {
            /// Gyro Calibration Summation
            {
                sensors_event_t event;
                bno.getEvent(&event);
                ang_basis[0] +=
                    (int)event.orientation.x; // write orientation angles to array
                ang_basis[1] += (int)event.orientation.y;
                ang_basis[2] += (int)event.orientation.z;
            }
        }

        // take average
        ang_basis[0] /= calibrate_length;
        ang_basis[1] /= calibrate_length;
        ang_basis[2] /= calibrate_length;

        // correct average
        //  Correct average angles
        for (int i = 0; i < 3; i++) {
            if (ang_basis[i] > 180.0) {
                ang_basis[i] -= 360.0;
            } else if (ang_basis[i] < -180.0) {
                ang_basis[i] += 360.0;
            }
        }
    }
    // setup altimeter
    {
        // set starting altitude
        alt0 = 0;
        altitude = 0;
        avg_alt = 0;
        int num_points = ALT_CAL_SAMPLES;
        for (int i = 0; i < num_points; i++) {
            alt0 += baro.getHeightCentiMeters() / 30.48;
            delay(100);
        }
        alt0 /= num_points; // normalize to the number of samples collected
    }
    // Setup Accelerometer
    {
        for (int x = 0; x < calibrate_length; x++) {
            // Read raw values
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
    playTones(buzzerPin, song, 7);
    return 1;
}

float mean(float arr[]) {
    int counter = 0;
    for (int i = 0; i <= AVG_CNT; i++) {
        counter += arr[i];
    }
    return counter / static_cast<float>(AVG_CNT);
}

void do_ignite() {
    delay(DECELERATION_DELAY + IGNITION_DELAY);
    digitalWrite(SUSTAINER_PIN, HIGH);
    statusFlags.bit.igniter_fired = 1;
}
void do_main() {
    digitalWrite(MAIN_PIN, LOW);
    statusFlags.bit.main_deployed = 1;
}
void do_drouge() {
    digitalWrite(DROUGE_PIN, LOW);
    statusFlags.bit.drogue_deployed = 1;
}

void start_flight() {
    // Setup Burnout Code
    startTime = millis();
    booster_on = true;
    digitalWrite(CAMERA_PIN, HIGH);
    statusFlags.bit.camera_on = 1;
}

void end_flight() {
    // Setup Burnout Code
    startTime = millis();
    booster_on = false;
    digitalWrite(CAMERA_PIN, LOW);
    statusFlags.bit.camera_on = 0;
}

bool allAltSamplesBelow(float refHeight) {
    for (size_t i = 0; i < AVG_CNT; ++i) {
        if (alt_trend[i] > refHeight) {
            return false;
        }
    }
    return true;
}

void writeLogLine(void) {
    logFile.print(millis());
    logFile.print(",");
    logFile.print(static_cast<int>(recoveryState));
    logFile.print(",");
    logFile.print(static_cast<int>(ignitionState));
    logFile.print(",");
    logFile.print(currentData.altFt);
    logFile.print(",");
    logFile.print(currentData.altAvgFt);
    logFile.print(",");
    logFile.print(currentData.linAccel);
    logFile.print(",");
    logFile.print(currentData.linAccelAvg);
    logFile.print(",");
    logFile.print(currentData.theta_x);
    logFile.print(",");
    logFile.print(currentData.theta_y);
    logFile.print(",");
    logFile.print(currentData.theta_z);
    logFile.print(",");
    logFile.print(currentData.inclination);
    logFile.print(",");
    logFile.print(statusFlags.reg);
    logFile.println();
    logFile.flush();
}

// Setup for state machine
void setup() {
    // Startup beep
    pinMode(buzzerPin, OUTPUT);
    tone(buzzerPin, 1000);
    delay(500);
    noTone(buzzerPin);

    // enable serial if needed
    if constexpr (__debug_enabled || Mode == 2)
        Serial.begin(115200);

    // init globals
    statusFlags.reg = 0;
    memset(&currentData, 0, sizeof(TelemetryEvent));

    // Relay Setup
    //  WRITE HIGH STATES before setting pinmodes for safety to prevent race. If we write
    //  after PINMODE, then there is a brief window when the pin is low, which is very
    //  bad.
    digitalWrite(MAIN_PIN, HIGH);
    digitalWrite(DROUGE_PIN, HIGH);
    digitalWrite(SUSTAINER_PIN, LOW);
    digitalWrite(CAMERA_PIN, LOW);

    pinMode(MAIN_PIN, OUTPUT);
    pinMode(DROUGE_PIN, OUTPUT);
    pinMode(SUSTAINER_PIN, OUTPUT);
    pinMode(CAMERA_PIN, OUTPUT);

    // IMU
    bno.begin();
    bno.setExtCrystalUse(false);
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    DEBUG_PRINT("Sensors Initialized");

    // Altimiter
    baro.init();

    // SD Card
    // initialize datalogging
    if (!SD.begin(chipSelect)) {
        DEBUG_PRINT("Card failed, or not present");
        tone(buzzerPin, 300, 2000);
        while (1)
            ;
    }
    logFile = SD.open("Flight.csv", FILE_WRITE);
    if (!logFile) {
        DEBUG_PRINT("File open failed");
        tone(buzzerPin, 500, 2000);
        while (1)
            ; // permanent spin loop if we couldn't initialize data for logging
    }
    // header
    logFile.println(
        "time_ms,rec_state,ign_state,lin_accel,lin_accel_avg,alt_ft,alt_avg_ft,"
        "theta_x,theta_y,theta_z,inclination,flags");

    // fail case
    if (calibrate() == 0) {
        while (1)
            ;
    }
}

void loop() {
    delay(FSM_SPEED); // Add a delay of 1 second (adjust as needed)
    // variable assignment: {acceleration, altitude, altitude_trend, accel_trend, gyro
    // tilt}
    if (currentData.altAvgFt > max_height) {
        max_height = currentData.altAvgFt;
    }
    if (Mode == 0) {
        String s_data[2];
        read_telemetry_test(s_data);
        currentData.linAccel = s_data[0].toInt(); // accel
        currentData.altFt = s_data[1].toInt();    // alt
    } else {
        read_telemetry();
    }

    alt_trend[mem_pointer] = currentData.altFt;
    acc_trend[mem_pointer] = currentData.linAccel;
    mem_pointer += 1;
    mem_pointer = mem_pointer % AVG_CNT;

    currentData.altAvgFt = mean(alt_trend);    // alt average
    currentData.linAccelAvg = mean(acc_trend); // acc average
    String printout;
    // Update state machine Recovery State
    switch (recoveryState) {
        // State to represent the rocket pre launch
        case RecoveryState::IDLE_REC: {
            DEBUG_PRINT("recovery state: IDLE_REC");
            // check for change in acceleration and set state to detecting launch
            if (currentData.linAccelAvg >= 0 && currentData.altAvgFt > ARM_ALT) {
                recoveryState = RecoveryState::DETECT_LAUNCH_REC;
                start_flight(); // start counting booster burn as soon as we detect
                                // acceleration.
            }
            break;
        }
        // state to represent the rocket post launch (can mistake heavy jostling for
        // launch)
        case RecoveryState::DETECT_LAUNCH_REC: {
            DEBUG_PRINT("recovery state: DETECT_LAUNCH_REC");
            // Placeholder condition for confirming launch
            // 1 = altitude
            if (currentData.altAvgFt >= LAUNCH_FLOOR) {
                // height floor reached
                recoveryState = RecoveryState::VALID_FLIGHT_REC;
            }
            // disarm if all tracked samples fall below the arm height
            if (allAltSamplesBelow(ARM_ALT)) {
                recoveryState = RecoveryState::IDLE_REC;
                end_flight();
            }
            break;
        }
        // State to represent further confirmation of launch with height floor
        case RecoveryState::VALID_FLIGHT_REC: {
            DEBUG_PRINT("recovery state: VALID_FLIGHT_REC");
            // Placeholder condition for detecting apogee and triggering drouge deploy
            if (currentData.altAvgFt <
                max_height - APPOGEE_DROP_THRESHOLD) { // if trending down
                recoveryState = RecoveryState::DEPLOY_DROGUE_REC;
            }
            break;
        }
        // State to trigger drogue deploy at apogee
        case RecoveryState::DEPLOY_DROGUE_REC: {
            DEBUG_PRINT("recovery state: DEPLOY_DROGUE_REC");
            do_drouge();
            if (currentData.altAvgFt <= MAIN_CHUTE_ALT) {
                recoveryState = RecoveryState::DEPLOY_MAIN_REC;
            }
            break;
        }
        // State to trigger main deploy at floor
        case RecoveryState::DEPLOY_MAIN_REC: {
            DEBUG_PRINT("recovery state: DEPLOY_MAIN_REC");
            do_main();
            // END STATE
            break;
        }
    }

    // Update state machine for Ignition State
    switch (ignitionState) {
        // rocket idle state
        case IgnitionState::IDLE_IGN: {
            DEBUG_PRINT("igniter_state: IDLE_IGN");
            // movement of rocket is the current assumption for launch
            if (currentData.linAccelAvg >= 0 && currentData.altAvgFt > ARM_ALT) {
                ignitionState = IgnitionState::DETECT_LAUNCH_IGN;
            }
            break;
        }
        // pre-official launch to account for false launches
        case IgnitionState::DETECT_LAUNCH_IGN: {
            DEBUG_PRINT("igniter state: DETECT_LAUNCH_IGN");
            // Placeholder condition for confirming launch
            if (currentData.altAvgFt >= LAUNCH_FLOOR) { // floor reached
                ignitionState = IgnitionState::VALID_FLIGHT_IGN;
            }
            // detect false launch  (we dropped the rocket)
            if (allAltSamplesBelow(ARM_ALT)) { // if not tredning anywhere
                ignitionState = IgnitionState::IDLE_IGN;
            }
            break;
        }
        // once valid flight established
        case IgnitionState::VALID_FLIGHT_IGN: {
            DEBUG_PRINT("igniter state: VALID_FLIGHT_IGN");
            // if any tilt exists initiate lockout to avoid Missiling
            unsigned long currentTime = millis();
            if (currentData.inclination >= LOCKOUT_DEGREE) { //
                ignitionState = IgnitionState::LOCKOUT_IGN;
            }
            // detect time to ignite based on expected burn time if launch has started
            else if (booster_on && currentTime - startTime > BOOSTER_BURN) {
                ignitionState = IgnitionState::IGNITE_IGN;
            }
            break;
            ignitionState = IgnitionState::LOCKOUT_IGN;
            break;
        }
        case IgnitionState::LOCKOUT_IGN: {
            DEBUG_PRINT("igniter state: LOCKOUT_IGN");
            // END STATE
            break;
        }
        case IgnitionState::IGNITE_IGN: {
            DEBUG_PRINT("igniter state: IGNITE_IGN");
            do_ignite();
            // END STATE
            break;
        }
    }
    // Datalogging:
    writeLogLine();
}
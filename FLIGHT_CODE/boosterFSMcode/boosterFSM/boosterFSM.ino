#include <Adafruit_BNO055.h>
#include <LoRa.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// uncomment to enable debug printing, must define BEFORE debug.h
// #define DEBUG
#include "debug.h"
#include "IntersemaBaro.h"

// TODO: check which angle is roll in rocket

// Define the possible states for each set
enum class RecoveryState {
    IDLE_REC,
    DETECT_LAUNCH_IGN,
    ASCENT_REC,
    DESCENT_REC,
};

struct TelemetryEvent {
    float altFt;
    float altAvgFt;
    float theta_x;
    float theta_y;
    float theta_z;
    float inclination;
};

union StatusFlags {
    struct {
        uint8_t separated : 1;
        uint8_t main_deployed : 1;
        uint8_t reserved : 6;
    } bit;
    uint8_t reg;
};

//%%%%%%%%%%%%%%%%%%%%%%%% FLIGHT OPS PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%//

#define FSM_SPEED 0                     // delay per FSM iteration [helps for testing 0 for launch] (ms).
#define ARM_ALT 5.0                     // Altitude to arm system into ascent detection mode (ft).
#define LAUNCH_FLOOR 10.0               // Altitude to use to arm into from ascent detect to ascent mode (ft).
#define DECELERATION_DELAY 500          // delay from when we detect motor burnout before sep (ms).
#define BOOSTER_BURN 1000               // duration of expected burn (ms).
#define DESC_DROP_THRESH 20.0           // -change in alt from apogee before entering descent phase (ft).
#define MAIN_DEPLOYMENT_ALTITUDE 50.0   // Height above ground to trigger parachute deployment (ft).
#define ALT_CAL_SAMPLES 20              // number of readings to take while claibrating 0-point.
#define AVG_CNT 4                       // number of sensor samples to average to smooth noise. MUST BE <= 10.
#define MAIN_NEED_SEP_DELAY 500         // Delay in ms between emergency separation and booster main chute 
#define MAIN_FROM_APOGEE_DELAY 0        // Delay in ms before main chute deploys (assumed zero unless FlOps)

// PINS
#define MAIN_PIN 5
#define SEP_PIN 7

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
int chipSelect = 4;

//%%%%%%%%%%%%%%%%%%%%%%%% FSM SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
// Define the initial state for each set
RecoveryState state = RecoveryState::IDLE_REC;
int calibrate_length =
    500; // sets how many iterations of average sensor readings we take on launch pad
StatusFlags statusFlags;

// memory
int max_height = 0;
int base_height = 0;
unsigned long startTime;
bool booster_on = false;
// TODO::ENSURE THAT ACCELERATION IS change in height over time NOT absolute accel to
// prevent issues with sideways travel ENSURE THIS IS ALTITUDE ABOVE GROUND NOT FROM SEA
float alt_trend[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int mem_pointer = 0;
float acc_trend[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

File logFile;
TelemetryEvent currentData;

//%%%%%%%%%%%%%%%%%%%%%%%% PIN SETUP %%%%%%%%%%%%%%%%%%%%%%%%//
// pin settings
int buzzerPin = 9;

// sensor settings
const int Mode = 1; // 1:sensor 2: test from laptop

void beep(uint8_t cnt) {
    for (uint8_t i = 0; i < cnt; ++i) {
        digitalWrite(buzzerPin, HIGH);
        delay(750);
        digitalWrite(buzzerPin, LOW);
        delay(750);
    }
}

void fastBeep(uint8_t cnt) {
    for (uint8_t i = 0; i < cnt; ++i) {
        digitalWrite(buzzerPin, HIGH);
        delay(100);
        digitalWrite(buzzerPin, LOW);
        delay(100);
    }
}

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
    double alpha = acos(sqrt(v_y * v_y + v_z * v_z) / v_magnitude);

    // Convert radians to degrees for the final result
    float alphaDegrees = abs(90 - alpha * RAD_TO_DEG);

    return alphaDegrees;
}

void read_telemetry() {
    // altitude
    {
        DEBUG_PRINT(baro.getHeightCentiMeters());
        currentData.altFt = baro.getHeightCentiMeters() / 30.48 - alt0;
        DEBUG_PRINT("Displayed altitude: ");
        DEBUG_PRINT(currentData.altFt);
        // Serial.print("Read altitude: ");
        // Serial.println(data[1] + alt0);
        // Serial.print("Zero altitude: ");
        // Serial.println(alt0);
    }
    // gyro
    {
        float ang[3] = {0, 0, 0};
        sensors_event_t event;
        bno.getEvent(&event);
        currentData.theta_x = calculateAngleDifference(
            ang_basis[0], (int)event.orientation.x); // write orientation angles to array
        currentData.theta_y =
            calculateAngleDifference(ang_basis[1], (int)event.orientation.y);
        currentData.theta_z =
            calculateAngleDifference(ang_basis[2], (int)event.orientation.z);
        delay(BNO055_SAMPLERATE_DELAY_MS);
        // currentData.inclination = (ang[2]*ang[2])+(ang[3]*ang[3]);
        currentData.inclination = calculateInclination(currentData.theta_y, currentData.theta_z);
        // Serial.print("bad_tilt: ");
        // Serial.println(data[4]);
    }
}

void do_sep() {
    delay(DECELERATION_DELAY);
    digitalWrite(SEP_PIN, LOW);
    statusFlags.bit.separated = 1;
    DEBUG_PRINT(" SEPARATION !!!!!!!!!!!!!!!!!!!!!!!!!");
}

void do_main() {
    DEBUG_PRINT("deploying main chute");
    if (!statusFlags.bit.separated) {
        digitalWrite(SEP_PIN, LOW);
        delay(MAIN_NEED_SEP_DELAY);
    }
    delay(MAIN_FROM_APOGEE_DELAY);
    statusFlags.bit.main_deployed = 1;
    digitalWrite(MAIN_PIN, LOW);
}

void start_flight() {
    // Setup Burnout Code
    startTime = millis();
    booster_on = true;
}

void end_flight() {
    // Setup Burnout Code
    startTime = millis();
    booster_on = false;
}

void calibrate() {
    { // Setup Burnout Code
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

        // Correct average
        // Correct average angles
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
        for (int i = 0; i < ALT_CAL_SAMPLES; i++) {
            alt0 += baro.getHeightCentiMeters() / 30.48;
            delay(100);
        }
        alt0 /= static_cast<float>(ALT_CAL_SAMPLES); // normalize to the number of samples collected
    }
}

float mean(float arr[]) {
    int counter = 0;
    for (int i = 0; i <= AVG_CNT; i++) {
        counter += arr[i];
    }
    return counter / static_cast<float>(AVG_CNT);
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
    logFile.print(static_cast<int>(state));
    logFile.print(",");
    logFile.print(currentData.altFt);
    logFile.print(",");
    logFile.print(currentData.altAvgFt);
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
    pinMode(buzzerPin, OUTPUT);
    beep(1);

    // enable serial if needed
    if constexpr (__debug_enabled || Mode == 2)
        Serial.begin(115200);

    memset(&currentData, 0, sizeof(TelemetryEvent));
    statusFlags.reg = 0;

    // initialize relay pins
    digitalWrite(SEP_PIN, HIGH);
    digitalWrite(MAIN_PIN, HIGH);
    pinMode(SEP_PIN, OUTPUT);
    pinMode(MAIN_PIN, OUTPUT);

    // IMU
    bno.begin();
    bno.setExtCrystalUse(false);
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    // Altimeter
    baro.init();

    // SD Card
    // initialize datalogging
    if (!SD.begin(chipSelect)) {
        DEBUG_PRINT("SD init failed");
        beep(3);
        while (1)
            ; // permanent spin loop if we couldn't initialize data for logging
    }
    logFile = SD.open("Flight.csv", FILE_WRITE);
    if (!logFile) {
        DEBUG_PRINT("File open failed");
        beep(4);
        while (1)
            ; // permanent spin loop if we couldn't initialize data for logging
    }
    // header
    logFile.println("time_ms,state,alt_ft,alt_avg_ft,theta_x,theta_y,theta_z,inclination,flags");

    calibrate();
    fastBeep(3);
}

void loop() {
    delay(FSM_SPEED); // Add a delay of 1 second (adjust as needed)
    // variable assignment: {acceleration, altitude, altitude_mean, accel_mean,
    // abs_gyro_tilt}
    if (currentData.altAvgFt > max_height) {
        max_height = currentData.altAvgFt;
    }
    if constexpr (Mode == 0) {
        String s_data[2];
        read_telemetry_test(s_data);
        currentData.altFt = s_data[1].toInt(); // alt
    } else {
        read_telemetry();
    }

    alt_trend[mem_pointer] = currentData.altFt;
    mem_pointer += 1;
    mem_pointer = mem_pointer % AVG_CNT;

    currentData.altAvgFt = mean(alt_trend); // alt average
    // TODO::test

    // Update state machine
    switch (state) {
        case RecoveryState::IDLE_REC: { // State to represent the rocket pre launch
            DEBUG_PRINT("IDLE_REC");
            // movement of rocket is the current assumption for launch
            if (currentData.altAvgFt > ARM_ALT) {
                state = RecoveryState::DETECT_LAUNCH_IGN;
                start_flight();
            }
            break;
        }

        // State to represent the rocket post launch (can mistake heavy jostling for
        // launch)
        case RecoveryState::DETECT_LAUNCH_IGN: {
            DEBUG_PRINT("DETECT_LAUNCH_IGN");
            // Serial.println(String(data[1]));
            // Placeholder condition for confirming launch
            // index[1] = altitude
            // Check if height floor reached and set to valid launch
            if (currentData.altAvgFt >= LAUNCH_FLOOR) {
                state = RecoveryState::ASCENT_REC;
            }
            // detect false launch
            // index[2] = altitude_trend
            if (allAltSamplesBelow(ARM_ALT)) {
                // if altitude is not changing
                state = RecoveryState::IDLE_REC;
                end_flight();
            }
            break;
        }

        // State to represent further confirmation of launch with height floor
        case RecoveryState::ASCENT_REC: {
            DEBUG_PRINT("ASCENT_REC");
            // Serial.println(data[2] - max_height);
            //  index[2] = atitude trend
            //  check if altitude is trending downwards
            if (currentData.altAvgFt <=  max_height - DESC_DROP_THRESH) {
                state = RecoveryState::DESCENT_REC;
            }
            unsigned long currentTime = millis();
            if (booster_on && currentTime - startTime > BOOSTER_BURN) {
                do_sep();
            }
            break;
        }

        case RecoveryState::DESCENT_REC: {
            do_main();
        }
    }

    writeLogLine();
}

// TESTING PROGRAM FOR READING PACKET CAPABILITIES
// TODO: make sure a single weird packet/reading doesn't trigger anything for the state machine

// struct for packet
typedef struct
{
    uint16_t thetaGyro; // 4
    uint16_t phiGyro;   // 4
    uint16_t psiGyro;   // 4
    uint16_t xAcc;      // 4
    uint16_t yAcc;      // 4
    uint16_t zAcc;      // 4

} __attribute__((packed)) IMU;

typedef struct
{
    char header = 0x55; // 1
    char padding0;      // 1
    char stage;         // 1
    char padding1;      // 1
    uint32_t latitude;  // 4
    char padding2;      // 1
    uint32_t longitude; // 4
    char padding3;      // 1
    uint32_t altitude;  // 4
    char padding4;      // 1
    uint32_t altTrend;
    char padding5;
    IMU imuPacket; // 24
} __attribute__((packed)) Packet;

// initialize packet size based on struct
const int intended_packet_size = sizeof(Packet);
Packet read_telemetry()
{
    char raw_data[50];
    int data[3];

    while (!Serial.available())
        ;

    Serial.readBytesUntil("\n", raw_data, 100);
    int data_index = 0;
    uint32_t buffered_value = 0;
    for (int i = 0; i < strlen(raw_data); i++)
    { // loop through raw chars of serial data input
        if (raw_data[i] == ',')
        {                                      // when we see a comma reset the number we are counting
            data[data_index] = buffered_value; // store the summed up value
            data_index += 1;                   // move on to collecting the next value
            buffered_value = 0;                // reset the buffered value we are counting
        }
        buffered_value = buffered_value * 10;       // shift left by multiple of 10
        buffered_value += ((int)raw_data[i] - '0'); // store the next char in the string as an int
    }
    IMU imuP;
    imuP.thetaGyro = 0;
    imuP.phiGyro = 0;
    imuP.psiGyro = 0;
    imuP.xAcc = 0;
    imuP.yAcc = 0;
    imuP.zAcc = data[0];

    // set up transmission Packet with fake data
    Packet packet;
    packet.stage = '1';
    packet.latitude = 0;
    packet.longitude = 10;
    packet.altitude = data[1];
    packet.altTrend = data[2];
    packet.imuPacket = imuP;
    return packet;
}

// Define the LED pins
const int ledPin1 = 2;
const int ledPin2 = 3;
const int ledPin3 = 4;

// Setup for state machine
void setup()
{
  // Set LED pins as output
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);

    // Attempt Lora setup
    Serial.begin(115200);
    while (!Serial)
        ;
}

void displayBinaryOnLEDs(int decimalNumber)
{
  // Extract individual bits and display on LEDs
  digitalWrite(ledPin1, (decimalNumber & 0b001) ? HIGH : LOW);
  digitalWrite(ledPin2, (decimalNumber & 0b010) ? HIGH : LOW);
  digitalWrite(ledPin3, (decimalNumber & 0b100) ? HIGH : LOW);
}

void loop()
{
    // variable assignment: {acceleration index, altitude, altitude_trend}
    Packet telemetry = read_telemetry();
    displayBinaryOnLEDs(telemetry.altitude);
}
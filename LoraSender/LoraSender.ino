
#include <LoRa.h>

/*
stage label bit (1 bit)
status (1 byte, chars)
GPS data —> 2 32 bit long  (lat & longitude)
altimeter —> int 32 bit
IMU (gyro & accelerometer) —> 6 16 bit integers 
*/

//struct for packet
typedef struct {
  int16_t xGyro; //4
  int16_t yGyro; //4
  int16_t zGyro; //4
  int16_t xAcc; //4
  int16_t yAcc; //4
  int16_t zAcc; //4

} __attribute__((packed)) IMU;

typedef struct {
  char header = 0x55; //1
  char padding0; //1
  char stage; //1
  char padding1; //1
  double latitude; //4
  char padding2; //1
  double longitude; //4
  char padding3; //1
  double altitude; //4
  char padding4; //1
  IMU imuPacket; // 24
} __attribute__((packed)) Packet;

//struct for transmission
const int intended_packet_size = sizeof(Packet);

void setup() {
  //Lora Setup
  Serial.begin(115200);
  while (!Serial);
  if (!LoRa.begin(915E6)) {
    while (1);
  }
}

void send_to_lora(uint8_t * packet) {
  //writing with packet
  LoRa.beginPacket();
  LoRa.write(packet, intended_packet_size);
  LoRa.endPacket();
}


void loop() {

  //set up IMU packet
  IMU imuP;
  imuP.xGyro = 0;
  imuP.yGyro = 0;
  imuP.zGyro = 0;
  imuP.xAcc = 0;
  imuP.yAcc = 0;
  imuP.zAcc = 0;

  //set up transmission Packet
  Packet packet;
  packet.stage = '1';
  packet.latitude = 0;
  packet.longitude = 10;
  packet.altitude = 0;
  packet.imuPacket = imuP;

  //convert to uint8_t packet
  uint8_t * packet_addr = (uint8_t *)(&packet);

  //send to lora function
  send_to_lora(packet_addr);
}

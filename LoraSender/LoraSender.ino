
#include <LoRa.h>

/*
stage label bit (1 bit)
status (1 byte, chars)
GPS data —> 2 32 bit long  (lat & longitude)
altimeter —> int 32 bit
IMU (gyro & accelerometer) —> 6 16 bit integers 
*/

//struct for transmission
int packet_size = 54;

typedef struct {
  char stage;
  char padding1;
  long latitude;
  char padding2;
  long longitude;
  char padding3;
  int altimeter;
  char padding4;
  IMU imuPacket;
} __attribute__((packed)) Packet;

//struct for packet
typedef struct {
  int xGyro;
  char padding1;
  int yGyro;
  char padding2;
  int zGyro;
  char padding3;
  int xAcc;
  char padding4;
  int yAcc;
  char padding4;
  int zAcc;

} __attribute__((packed)) IMU;

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
  LoRa.write(packet, packet_size);
  LoRa.endPacket();
}


void loop() {

  //set up IMU packet
  IMU imuP;
  imuPacket.xGyro = 0;
  imuPacket.yGyro = 0;
  imuPacket.zGyro = 0;
  imuPacket.xAcc = 0;
  imuPacket.yAcc = 0;
  imuPacket.zAcc = 0;

  //set up transmission Packet
  Packet packet;
  packet.stage = '1';
  packet.latitude = 0;
  packet.longitude = 0;
  packet.altimeter = 0;
  packet.imuPacket = imuP;
z
  //convert to uint8_t packet
  uint8_t * packet_addr = (uint8_t *)(&packet);

  //send to lora function
  send_to_lora(packet_addr);
}

// #include <TinyGPS.h>
#include <LoRa.h>
// #include "IntersemaBaro.h"

/*
stage label bit (1 bit)
status (1 byte, chars)
GPS data —> 2 32 bit long  (lat & longitude)
altimeter —> int 32 bit
IMU (gyro & accelerometer) —> 6 16 bit integers 
*/

//struct for packet
typedef struct {
  uint16_t xGyro; //4
  uint16_t yGyro; //4
  uint16_t zGyro; //4
  uint16_t xAcc; //4
  uint16_t yAcc; //4
  uint16_t zAcc; //4

} __attribute__((packed)) IMU;

typedef struct {
  char header = 0x55; //1
  char padding0; //1
  char stage; //1
  char padding1; //1
  uint32_t latitude; //4
  char padding2; //1
  uint32_t longitude; //4
  char padding3; //1
  uint32_t altitude; //4
  char padding4; //1
  IMU imuPacket; // 24
} __attribute__((packed)) Packet;

//struct for transmission
const int intended_packet_size = sizeof(Packet);
char buffer[intended_packet_size];

void setup() {
  //set up serial output
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");
  //set up LoRa
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void send_to_lora(uint8_t * packet) {
  //writing with packet
  LoRa.beginPacket();
  LoRa.write(packet, intended_packet_size);
  LoRa.endPacket();
}

//unpack struct

void loop() {
  // try to parse packet
  int actual_packet_size = LoRa.parsePacket();

  //packet size is 16
  if (actual_packet_size==intended_packet_size) {
    // received a packet
    Serial.println("Received packet ");

    char first = (char)LoRa.read();
    //the header is always 0x55
    if(first == 0x55) {
      buffer[0] = first;
      //index
      int i = 1;
      //read into buffer
      while(i<intended_packet_size) {
        buffer[i]=(char)LoRa.read();
        ++i;
      }
      //parse the buffer into packet
      Packet * packet_ptr = (Packet *) buffer;
      //output lat lon 
      Serial.print("lat: ");
      Serial.println(packet_ptr->latitude);
      Serial.print("lon: ");
      Serial.println(packet_ptr->longitude);
      Serial.print("altitude: ");
      Serial.println(packet_ptr->altitude);
    }
  }
}

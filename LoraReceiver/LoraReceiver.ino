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
  LoRa.write(packet, packet_size);
  LoRa.endPacket();
}

//unpack struct
/*
void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();

  //packet size is 16
  if (packetSize==packet_size) {
    // received a packet
    Serial.println("Received packet ");
    

    char stage = (char)LoRa.read();
    //the header is always 0x55
    if(first == 0x55) {
      buffer[0] = first;
      int i=1;
      //read into buffer
      while(i<16) {
        buffer[i]=(char)LoRa.read();
        ++i;
      }
      //parse the buffer into packet
      Packet * packet_ptr = (Packet *) buffer;
      //output lat lon 
      Serial.print("lat: ");
      Serial.println(packet_ptr->lat);
      Serial.print("lon: ");
      Serial.println(packet_ptr->lon);
      Serial.print("altitude: ");
      Serial.println(packet_ptr->altitude);
    }

  }
  */
}
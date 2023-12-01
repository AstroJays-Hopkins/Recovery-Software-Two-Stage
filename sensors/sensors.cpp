#include <TinyGPS.h>
#include <SD.h>
#include <LoRa.h>
#include "MPU6050_6Axis_MotionApps20.h"

typedef struct {
  uint16_t xGyro; //4
  uint16_t yGyro; //4
  uint16_t zGyro; //4
  uint16_t xAcc; //4
  uint16_t yAcc; //4
  uint16_t zAcc; //4

  void to_string()

} __attribute__((packed)) IMU;

typedef struct {
  char header = 0x55; //1
  char padding0; //1
  char stage; //1
  char padding1; //1
  RawDegrees latitude; //4
  char padding2; //1
  RawDegrees longitude; //4
  char padding3; //1
  uint32_t altitude; //4
  char padding4; //1
  IMU imuPacket; // 24
} __attribute__((packed)) Packet;

const int intended_packet_size = sizeof(Packet);

volatile bool mpuInterrupt = false;

int getGPS(TinyGPS& gps, Serial& ss, Packet& output){
    while (ss.available() > 0)
        gps.encode(ss.read);

    if (gps.location.isUpdated()){
        output.longitude = gps.location.rawLat();
        output.latitude = gps.location.rawLat();
        return 1; 
    }
    else return 0; 
}

int writeSDCard(Serial& ss, File& myfile, Packet& output){
    if(!SD.begin(4)){
        Serial.print("initialization failed"); 
        return 0; 
    }

    myFile = SD.open("rocket_data.txt", FILE_WRITE); 
    if(!myFile){
        Serial.print("file open failed"); 
        return 0; 
    }

    return 1; 
}

void send_to_lora(uint8_t * packet) {
  //writing with packet
  LoRa.beginPacket();
  LoRa.write(packet, intended_packet_size);
  LoRa.endPacket();
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void get_bmp_data(MPU6050& mpu, Packet& output){
    int16_t ax, ay, az;

    int16_t gx, gy, gz;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        output.
    }
}
#include <TinyGPS.h>
#include <SD.h>
#include <LoRa.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>


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
  RawDegrees latitude; //4
  char padding2; //1
  RawDegrees longitude; //4
  char padding3; //1
  uint32_t altitude; //4
  char padding4; //1
  IMU imuPacket; // 24
  uint32_t mpu_X;
  uint32_t mpu_Y;
  uint32_t mpu_Z;
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

void get_mpu_data(MPU6050& mpu, Packet& output){
    int16_t ax, ay, az;

    int16_t gx, gy, gz;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        output.imuPacket.xAcc = ax;
        output.imuPacket.xAcc = ay;
        output.imuPacket.xAcc = az;
        output.imuPacket.xGyro = gx;
        output.imuPacket.yGyro = gy;
        output.imuPacket.zGyro = gz;
    }
}

void interruptRoutine()
{
    newDataAvailable = true;
}

void get_mpu_data(SFE_MMC5983MA& myMag, Packet& output){
    uint32_t rawValueX = 0;
    uint32_t rawValueY = 0;
    uint32_t rawValueZ = 0;
    if (newDataAvailable == true){
        newDataAvailable = false; // Clear our interrupt flag
        myMag.clearMeasDoneInterrupt(); // Clear the MMC5983 interrupt

        // Read all three channels simultaneously
        // Note: we are calling readFieldsXYZ to read the fields, not getMeasurementXYZ
        // The measurement is already complete, we do not need to start a new one
        myMag.readFieldsXYZ(&rawValueX, &rawValueY, &rawValueZ);

        output.mpu_X = rawValueX;
        output.mpu_Y = rawValueY;
        output.mpu_Z = rawValueZ;
    }
}



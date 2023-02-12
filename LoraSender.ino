
#include <TinyGPS.h>
#include <LoRa.h>
#include "IntersemaBaro.h"

//struct for packet
typedef struct {
  char header = 0x55;
  float lat;
  char padding1;
  float lon;
  char padding2;
  float altitude;
  char padding3;
} __attribute__((packed)) Packet;


//defining GPS serial
#define GPSSerial Serial1
#define Serial SERIAL_PORT_USBVIRTUAL

TinyGPS gps;

//altimiter set up
Intersema::BaroPressure_MS5607B baro(true);

// Altitude variables
float avg_alt;
float alt0;
float altitude;

// TV camera power management
#define TV_CAM_PIN 5     // pin to turn on live TV camera --- raise high
                          // to turn on camera.
#define TV_CAM_ON_ALTITUDE 5     // Altitude (in feet) at which camera turns on
int TV_cam_is_on = 0;                // 0 when camera is off; 1 when cam is on


static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPSSerial.available())
      gps.encode(GPSSerial.read());
  } while (millis() - start < ms);
}

void setup() {
  //Lora Setup
  Serial.begin(115200);
  while (!Serial);
  if (!LoRa.begin(915E6)) {
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  LoRa.setTxPower(20);
  Serial.println("Cranking up power!");
  //initalize baro
  baro.init();
  Serial.println("Initializing Baro!");

  //set starting altitude
  alt0 = 0;
  altitude = 0;
  avg_alt = 0;
  int num_points = 50;
  for (int i = 0; i < num_points; i++)
  {
    alt0 += baro.getHeightCentiMeters() / 30.48;
    delay(10);
  }
  alt0 /= num_points;

  //GPS startup
  GPSSerial.begin(9600);

  // Enable TV camera power control pin as output and set to low to
  // keep camera off.
  pinMode(TV_CAM_PIN, OUTPUT);
  digitalWrite(TV_CAM_PIN, LOW);
}


//format
//1 byte marker

//4 bytes of float lat
//1 byte of 

//4 bytes of float lon
//1 bytes of padding

//4 bytes of altitude
//1 byte of padding
//total 16 bytes

void send_to_lora(uint8_t * packet) {
  //writing with packet
  LoRa.beginPacket();
  LoRa.write(packet, 16);
  LoRa.endPacket();
}

//global variables for lat and lon
float flat = 0;
float flon = 0;
unsigned long age = 0;

void loop() {
  smartdelay(10);
  gps.f_get_position(&flat, &flon, &age);

  altitude = baro.getHeightCentiMeters() / 30.48 - alt0;
  avg_alt += (altitude - avg_alt) / 5;
 
  //pack the struct
  Packet packet;
  packet.lat = flat;
  packet.lon = flon;
  packet.altitude = avg_alt;

  // Turn on the TV camera if it is off and we've exceeded the
  // altitute it's supposed to turn on at
  if (!TV_cam_is_on && ((avg_alt - alt0) > TV_CAM_ON_ALTITUDE)) {
    digitalWrite(TV_CAM_PIN, HIGH);
    TV_cam_is_on = 1;
  }
  
  Serial.print("lat: ");
  Serial.println(packet.lat);
  Serial.print("lon: ");
  Serial.println(packet.lon);
  Serial.print("altitude: ");
  Serial.println(packet.altitude);
  
  //convert to uint8_t packet
  uint8_t * packet_addr = (uint8_t *)(&packet);

  //send to lora function
  send_to_lora(packet_addr);
}

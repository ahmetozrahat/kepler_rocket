#include "Arduino.h"
#include "LoRa_E22.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>

#define KAYSERI_SEA_LEVEL_PRESSURE 102550

#define LED_TX 5
#define LED_RX 6
#define LED_RUN 7

#define LORA_AUX 37
#define LORA_M0 35
#define LORA_M1 36

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;
TinyGPS gps;
LoRa_E22 e22ttl(&Serial1, LORA_AUX, LORA_M0, LORA_M1);

float flat, flon, temp, altitude;
unsigned long age;
int counter = 0;

struct Payload {
  int package_id;
  byte temp[4];
  byte altitude[4];
  byte gps_lat[4];
  byte gps_long[4];
  byte accl_x[4];
  byte accl_y[4];
  byte accl_z[4];
  byte gyro_x[4];
  byte gyro_y[4];
  byte gyro_z[4];
} payload;

static void smartdelay(unsigned long ms);

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);

  e22ttl.begin();

  // Pin mode setup
  pinMode(LED_TX, OUTPUT);
  pinMode(LED_RX, OUTPUT);
  pinMode(LED_RUN, OUTPUT);

  digitalWrite(LED_RUN, HIGH);

  // BMP180 initialization code
  if (!bmp.begin()) {
	  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	  while (1) {
      delay(10);
    }
  }

  // MPU6050 initialization code
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // Read GPS coordinate data.
  digitalWrite(LED_RX, HIGH);
  gps.f_get_position(&flat, &flon, &age);

  *(float*)(payload.gps_lat) = flat;
  *(float*)(payload.gps_long) = flon;

  delay(50);
  digitalWrite(LED_RX, LOW);
  
  smartdelay(200);

  // Read temperature data.
  temp = bmp.readTemperature();
  *(float*)(payload.temp) = temp;
    
  // Read altitude data.
  altitude = bmp.readAltitude(KAYSERI_SEA_LEVEL_PRESSURE);
  *(float*)(payload.altitude) = altitude;

  // Read acceleration and gyro data.
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  *(float*)(payload.accl_x) = a.acceleration.x;
  *(float*)(payload.accl_y) = a.acceleration.y;
  *(float*)(payload.accl_z) = a.acceleration.z;

  *(float*)(payload.gyro_x) = g.gyro.x;
  *(float*)(payload.gyro_y) = g.gyro.y;
  *(float*)(payload.gyro_z) = g.gyro.z;

  payload.package_id = ++counter;

  // digitalWrite(LED_TX, HIGH);
  // ResponseStatus rs = e22ttl.sendBroadcastFixedMessage(0x12, &payload, sizeof(Payload));
  
  Serial.print("Irtifa:");
  Serial.print(*(float*)payload.altitude);
  Serial.print(",");
  Serial.print("Sıcaklık:");
  Serial.print(*(float*)payload.temp);
  Serial.print(",");
  Serial.print("Enlem:");
  Serial.print(*(float*)payload.gps_lat);
  Serial.print(",");
  Serial.print("Boylam:");
  Serial.println(*(float*)payload.gps_long);

  delay(50);
  digitalWrite(LED_TX, LOW);
}

// Smart delay function for GPS
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial3.available())
      gps.encode(Serial3.read());
  } while (millis() - start < ms);
}

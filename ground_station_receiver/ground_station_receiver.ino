#include "Arduino.h"
#include "LoRa_E22.h"

#define ARDUINO_RX 5
#define ARDUINO_TX 4

#define AUX_PIN 3
#define M0_PIN 7
#define M1_PIN 6

// ---------- Arduino pins --------------
#include <SoftwareSerial.h>
SoftwareSerial mySerial(ARDUINO_TX, ARDUINO_RX); // Arduino RX <-- e22 TX, Arduino TX --> e22 RX
LoRa_E22 e22ttl(&mySerial, AUX_PIN, M0_PIN, M1_PIN); // AUX M0 M1
// -------------------------------------

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

void setup() {
  Serial.begin(9600);
  delay(500);
  
  e22ttl.begin();
}

void loop() {
  if (e22ttl.available() > 1) {
    ResponseStructContainer rsc = e22ttl.receiveMessage(sizeof(payload));
    struct Payload receivedPayload = *(Payload*) rsc.data;
    Serial.print("ID: ");
    Serial.print(receivedPayload.package_id);
    Serial.print("\tSicaklik: ");
    Serial.print(*(float*)receivedPayload.temp);
    Serial.print("\tIrtifa: ");
    Serial.print(*(float*)receivedPayload.altitude);
    Serial.print("\tEnlem: ");
    Serial.print(*(float*)receivedPayload.gps_lat, 6);
    Serial.print("\tBoylam: ");
    Serial.print(*(float*)receivedPayload.gps_long, 6);

    Serial.print("\tIvme X: ");
    Serial.print(*(float*)receivedPayload.accl_x);
    Serial.print("\tIvme Y: ");
    Serial.print(*(float*)receivedPayload.accl_y);
    Serial.print("\tIvme Z: ");
    Serial.print(*(float*)receivedPayload.accl_z);

    Serial.print("\tGyro X: ");
    Serial.print(*(float*)receivedPayload.gyro_x);
    Serial.print("\tGyro Y: ");
    Serial.print(*(float*)receivedPayload.gyro_y);
    Serial.print("\tGyro Z: ");
    Serial.print(*(float*)receivedPayload.gyro_z);
    
    Serial.println();
  }
}
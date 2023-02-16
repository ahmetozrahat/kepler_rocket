/**
  Kepler Roket Takımı Rocket Flight Computer Software
  Name: main_avionics_transmit
  Purpose: Code will collect altitude and acceleration
  data from the sensors and control the recovery system
  included in the rocket itself.
  @author Ahmet Ozrahat
  @version 1.1 02/01/2023
*/
#include "Arduino.h"
#include "LoRa_E22.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>

#define KAYSERI_SEA_LEVEL_PRESSURE 102550 // Sea level pressure for altitude calculation

#define LED_TX 5 // LED indicating a packed is transmitted.
#define LED_RX 6 // LED indicating a packed is received.
#define LED_RUN 8 // LED indicating the system is running.

#define LORA_AUX 37 // AUX pin for LoRa.
#define LORA_M0 35 // M0 conf. pin for LoRa.
#define LORA_M1 36 // M1 conf. pin for LoRa.

#define BUZZER_PIN 9 // Buzzer pin for notifications.

#define LORA_CHANNEL 0x12 // LoRa communication channel.

#define SAMPLE_RATE 5 // Sample rate in Hertz. The maximum is 5.

Adafruit_BMP085 bmp; // BMP180 pressure sensor initialization.
Adafruit_MPU6050 mpu; // MPU6050 IMU sensor initialization.
TinyGPS gps; // NEO6MV2 GPS initialization.
LoRa_E22 e22ttl(&Serial1, LORA_AUX, LORA_M0, LORA_M1); // LoRa initialization.

float flat, flon; // GPS data variables.
unsigned long age; // 
float temp, altitude, pressure; // Variables to get from pressure sensor.
int counter = 0; // Package id counter.

// Payload data stucture to send over LoRa WAN.
struct Payload {
  int package_id; // Package id
  byte temp[4]; // Temperature data
  byte altitude[4]; // Altitude
  byte pressure[4]; // Pressure
  byte gps_lat[4]; // GPS Latitude
  byte gps_long[4]; // GPS Longtitude
  byte accl_x[4]; // X axis acceleration
  byte accl_y[4]; // Y axis acceleration
  byte accl_z[4]; // Z axis acceleration
  byte gyro_x[4]; // X axis gyroscope
  byte gyro_y[4]; // Y axis gyroscope
  byte gyro_z[4]; // Z axis gyroscope
} payload;

// ----- Function definitions section -----

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);

// ----- Arduino Functions -----

void setup() {
  initialize_serial_ports();
  initialize_leds();
  initialize_buzzer();
  initialize_bmp180();
  initialize_mpu6050();
  initialize_lora();

  digitalWrite(LED_RUN, HIGH); // System is working.
}

/**
  Initialize serial ports in order to
  receive and transmit data through flight.
*/
void initialize_serial_ports() {
  Serial.begin(9600); // Start serial communication over Serial1 iwth 9600 baud rate.
  Serial3.begin(9600); // Serial communication for GPS over Serial3 with 9600 baud rate.
}

/**
  Initializes buzzer as output pin.
*/
void initialize_buzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
}

/**
  Setup pin modes of the indicator leds.
*/
void initialize_leds() {
  pinMode(LED_TX, OUTPUT);
  pinMode(LED_RX, OUTPUT);
  pinMode(LED_RUN, OUTPUT);

  digitalWrite(LED_RUN, HIGH);
}

/**
  Initialize BMP180 by checking it's address.
*/
void initialize_bmp180() {
  if (!bmp.begin()) {
    Serial.println("BMP180 bulunamadi, baglantilari kontrol ediniz.!");
    while (1) {
      delay(10);
    }
  }
}

/**
  Initialize MPU6050 by checking it's address.
*/
void initialize_mpu6050() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 bulunamadi, baglantilari kontrol ediniz.!");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

/**
  Initialize LoRa and get configuration from the device.
*/
void initialize_lora() {
  e22ttl.begin();
  ResponseStructContainer c;
	c = e22ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;

  if (c.status.code == 1) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(50);
      digitalWrite(BUZZER_PIN, LOW);
      delay(50);
    }
  }
}

void loop() {
  get_gps_sample();

  get_temp_sample();
    
  get_altitude_sample();

  get_pressure_sample();

  get_imu_sample();

  increment_counter();

  print_data_to_serial();

  transmit_payload();

  if (payload.altitude > 2000) {
    Serial.println("Irtifa ulasildi.");
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(50);
      digitalWrite(BUZZER_PIN, LOW);
      delay(50);
    }
  }
}

/**
  Get GPS data from MPU6050 and add it to the payload struct.
*/
void get_gps_sample() {
  digitalWrite(LED_RX, HIGH);
  gps.f_get_position(&flat, &flon, &age);

  *(float*)(payload.gps_lat) = flat;
  *(float*)(payload.gps_long) = flon;
  delay(50);
  digitalWrite(LED_RX, LOW);
  smartdelay(1100 - (SAMPLE_RATE * 200));  
}

/**
  Gets GPS data from MPU6050 and adds it to the payload struct.
*/
void get_temp_sample() {
  // Read temperature data.
  temp = bmp.readTemperature();
  *(float*)(payload.temp) = temp;
}

/**
  Gets altitude data from BMP180 and adds it to the payload struct.
*/
void get_altitude_sample() {
  // Read altitude data.
  altitude = bmp.readAltitude(KAYSERI_SEA_LEVEL_PRESSURE);
  *(float*)(payload.altitude) = altitude;
}

/**
  Gets sea level pressure data from BMP180 and adds it to the payload struct.
*/
void get_pressure_sample() {
  // Read pressure data.
  pressure = bmp.readSealevelPressure();
  *(float*)(payload.pressure) = pressure;
}

/**
  Gets IMU data from MPU6050 and adds it to the payload struct.
*/
void get_imu_sample() {
  // Read acceleration and gyro data.
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  *(float*)(payload.accl_x) = a.acceleration.x;
  *(float*)(payload.accl_y) = a.acceleration.y;
  *(float*)(payload.accl_z) = a.acceleration.z;

  *(float*)(payload.gyro_x) = g.gyro.x;
  *(float*)(payload.gyro_y) = g.gyro.y;
  *(float*)(payload.gyro_z) = g.gyro.z;
}

/**
  Transmits the payload struct over the LoRa network.
*/
void transmit_payload() {
  digitalWrite(LED_TX, HIGH);
  e22ttl.sendBroadcastFixedMessage(LORA_CHANNEL, &payload, sizeof(Payload));
  
  Serial.print("Package ID:");
  Serial.println(payload.package_id);

  delay(50);
  digitalWrite(LED_TX, LOW);
}

// ----- Helper Methods Section -----

/**
  Increments payload package ID from 1 to 255.
  Counter resets to 1 after 255.
*/
void increment_counter() {
  if (((counter + 1) % 256) == 0) {
    counter = 1;
    payload.package_id = counter;
  }else {
    counter++;
    payload.package_id = counter;
  }
}

/**
  Prints the gathered information from the sensors to Serial monitor.
*/
void print_data_to_serial() {
  Serial.print("ID: ");
  Serial.print(payload.package_id);
  Serial.print(", Sicaklik: ");
  Serial.print(*(float*)payload.temp, 2);
  Serial.print(", Irtifa: ");
  Serial.print(*(float*)payload.altitude);
  Serial.print(", Basinc: ");
  Serial.print(*(float*)payload.pressure);
  Serial.print(", Enlem: ");
  Serial.print(*(float*)payload.gps_lat, 6);
  Serial.print(", Boylam: ");
  Serial.print(*(float*)payload.gps_long, 6);
  Serial.println();
}

/**
  Smart delay function for GPS.

  @param ms required milliseconds.
*/
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial3.available())
      gps.encode(Serial3.read());
  } while (millis() - start < ms);
}

void printParameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
	Serial.print(F("NetID : "));  Serial.println(configuration.NETID, HEX);
	Serial.println(F(" "));
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRateDescription());
	Serial.println(F(" "));
	Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getSubPacketSetting());
	Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
	Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
	Serial.println(F(" "));
	Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
	Serial.print(F("TransModeTransContr: "));  Serial.print(configuration.TRANSMISSION_MODE.WORTransceiverControl, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORTransceiverControlDescription());
	Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
	Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
	Serial.print(F("TransModeEnabRepeat: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRepeater, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRepeaterModeEnableByteDescription());
	Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());

	Serial.println("----------------------------------------");
}

void printModuleInformation(struct ModuleInformation moduleInformation) {
	Serial.println("----------------------------------------");
	Serial.print(F("HEAD: "));  Serial.print(moduleInformation.COMMAND, HEX);Serial.print(" ");Serial.print(moduleInformation.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(moduleInformation.LENGHT, DEC);

	Serial.print(F("Model no.: "));  Serial.println(moduleInformation.model, HEX);
	Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
	Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
	Serial.println("----------------------------------------");
}
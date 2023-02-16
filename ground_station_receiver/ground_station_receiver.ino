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
LoRa_E22 e22ttl(&Serial1, AUX_PIN, M0_PIN, M1_PIN); // AUX M0 M1
// -------------------------------------

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);

struct Payload {
  int package_id;
  byte temp[4];
  byte altitude[4];
  byte pressure[4];
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

  ResponseStructContainer c;
	c = e22ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printParameters(configuration);

	ResponseStructContainer cMi;
	cMi = e22ttl.getModuleInformation();
	// It's important get information pointer before all other operation
	ModuleInformation mi = *(ModuleInformation*)cMi.data;

	Serial.println(cMi.status.getResponseDescription());
	Serial.println(cMi.status.code);

	printModuleInformation(mi);
}

void loop() {
  if (e22ttl.available() > 1) {
    ResponseStructContainer rsc = e22ttl.receiveMessage(sizeof(payload));
    struct Payload receivedPayload = *(Payload*) rsc.data;
    Serial.print(receivedPayload.package_id);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.altitude);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.pressure);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.temp);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.gps_lat, 6);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.gps_long, 6);
    Serial.print(",");
    
    Serial.print(*(float*)receivedPayload.accl_x);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.accl_y);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.accl_z);
    Serial.print(",");

    Serial.print(*(float*)receivedPayload.gyro_x);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.gyro_y);
    Serial.print(",");
    Serial.print(*(float*)receivedPayload.gyro_z);
    Serial.println(",");
  }
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
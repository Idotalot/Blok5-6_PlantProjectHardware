#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "sensors/TemperatureSensor.h"
#include "sensors/PhSensor.h"
 
// DS18B20 op eigen pin
#define SENSOR1_PIN 5
TemperatureSensor tempSensor(SENSOR1_PIN);
 
// pH sensor op analoge pin A0
#define PH_PIN A0
const float ADC_REF_VOLT = 5.0;   // referentiespanning van ADC (pas aan indien 3.3V board)
const int   ADC_RESOLUTION = 1023; // 10-bit ADC van Arduino Uno/Nano
const int   PH_SAMPLES = 10;       // aantal metingen middelen voor stabiliteit
PhSensor phSensor(PH_PIN, ADC_REF_VOLT, ADC_RESOLUTION, PH_SAMPLES);
 
// LoRaWAN ABP-sleutels
static const PROGMEM u1_t NWKSKEY[16] = { 0xF0, 0xE8, 0x88, 0xE6, 0xF9, 0xAD, 0x52, 0xEE, 0x98, 0x40, 0xFF, 0xBA, 0xC8, 0xD2, 0x81, 0x5D };
static const PROGMEM u1_t APPSKEY[16] = { 0xE3, 0xD0, 0xA3, 0xEA, 0x86, 0x14, 0xD5, 0x3C, 0x2B, 0x21, 0xDF, 0x9E, 0x89, 0x8A, 0xFE, 0x78 };
static const u4_t DEVADDR = 0x260B48AB; // Vervang met jouw DevAddr
 
void os_getArtEui(u1_t* buf) { }
void os_getDevEui(u1_t* buf) { }
void os_getDevKey(u1_t* buf) { }
 
// Payload: twee losse records van 3 bytes elk: [id][value_hi][value_lo]
// Record 1 = temperatuur, Record 2 = pH (beide x100, int16)
uint8_t mydata[6];
const uint8_t TEMP_SENSOR_ID = 1;
const uint8_t PH_SENSOR_ID = 2;
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;
 
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};
 
void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE"));
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    default:
      Serial.println(F("Other event"));
      break;
  }
}
 
void do_send(osjob_t* j) {
  float temp = tempSensor.readCelsius();
  Serial.print("Sensor 1 temperatuur: ");
  Serial.println(temp);
 
  if (temp == DEVICE_DISCONNECTED_C) {
    Serial.println(F("Sensor niet gevonden, versturen overgeslagen."));
    return;
  }
 
  // Lees en bereken pH
  float phVoltage = phSensor.readVoltage();
  float ph = phSensor.readPh();
  Serial.print("pH spanning (V): ");
  Serial.println(phVoltage, 3);
  Serial.print("Berekenede pH: ");
  Serial.println(ph, 2);

  int16_t tempVal = round(temp * 100);
  int16_t phVal = round(ph * 100);
  // Record 1: temperatuur
  mydata[0] = TEMP_SENSOR_ID;
  mydata[1] = highByte(tempVal);
  mydata[2] = lowByte(tempVal);
  // Record 2: pH
  mydata[3] = PH_SENSOR_ID;
  mydata[4] = highByte(phVal);
  mydata[5] = lowByte(phVal);
  Serial.println(tempVal);
 
//   Serial.print("Payload bytes: ");
  Serial.print(mydata[0]);
  Serial.print(" ");
  Serial.print(mydata[1]);
  Serial.print(" ");
  Serial.print(mydata[2]);
  Serial.print(" ");
  Serial.print(mydata[3]);
  Serial.print(" ");
  Serial.print(mydata[4]);
  Serial.print(" ");
  Serial.println(mydata[5]);
 
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
}
 
void setup() {
  Serial.begin(115200);
  Serial.println(F("Opstarten met 1 sensor..."));
 
  tempSensor.begin();
  tempSensor.setResolution(12);
  tempSensor.setWaitForConversion(true);
  // Stel pH kalibratiepunten in (pas aan naar jouw gemeten spanningen)
  phSensor.setCalibration(7.00, 1.882, 4.00, 1.982);
 
  os_init();
  LMIC_reset();
 
  #ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
  #else
  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif
 
  #if defined(CFG_eu868)
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);
  #elif defined(CFG_us915)
  LMIC_selectSubBand(1);
  #endif
 
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);
 
  do_send(&sendjob);
}
 
void loop() {
  os_runloop_once();
}
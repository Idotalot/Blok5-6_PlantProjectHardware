#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
 
// DS18B20 op eigen pin
#define SENSOR1_PIN 5
OneWire oneWire1(SENSOR1_PIN);
DallasTemperature sensor1(&oneWire1);
 
// LoRaWAN ABP-sleutels
static const PROGMEM u1_t NWKSKEY[16] = { 0xF0, 0xE8, 0x88, 0xE6, 0xF9, 0xAD, 0x52, 0xEE, 0x98, 0x40, 0xFF, 0xBA, 0xC8, 0xD2, 0x81, 0x5D };
static const PROGMEM u1_t APPSKEY[16] = { 0xE3, 0xD0, 0xA3, 0xEA, 0x86, 0x14, 0xD5, 0x3C, 0x2B, 0x21, 0xDF, 0x9E, 0x89, 0x8A, 0xFE, 0x78 };
static const u4_t DEVADDR = 0x260B48AB; // Vervang met jouw DevAddr
 
void os_getArtEui(u1_t* buf) { }
void os_getDevEui(u1_t* buf) { }
void os_getDevKey(u1_t* buf) { }
 
uint8_t mydata[3];  // 1 byte ID + 2 bytes temperatuur
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
  sensor1.requestTemperatures();
  delay(100);
  float temp = sensor1.getTempCByIndex(0);
  Serial.print("Sensor 1 temperatuur: ");
  Serial.println(temp);
 
  if (temp == DEVICE_DISCONNECTED_C) {
    Serial.println(F("Sensor niet gevonden, versturen overgeslagen."));
    return;
  }
 
  int16_t tempVal = round(temp * 100);
  mydata[0] = 1; // Sensor-ID
  mydata[1] = highByte(tempVal);
  mydata[2] = lowByte(tempVal);
  Serial.println(tempVal);
 
//   Serial.print("Payload: ID=");
  Serial.print(mydata[0]);
//   Serial.print(", Temp bytes=");
  Serial.print(mydata[1]);
  Serial.print(" ");
  Serial.println(mydata[2]);
 
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
 
  sensor1.begin();
 
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
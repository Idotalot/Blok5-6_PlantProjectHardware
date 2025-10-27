#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

// ------------------ DS18B20 Temperature ------------------
#define SENSOR1_PIN 5
OneWire oneWire1(SENSOR1_PIN);
DallasTemperature sensor1(&oneWire1);

// ------------------ pH Sensor ------------------
#define PH_PIN A0
const float PH_CAL_PH1 = 7.00;
const float PH_CAL_V1  = 1.882;
const float PH_CAL_PH2 = 4.00;
const float PH_CAL_V2  = 1.982;
const float ADC_REF_VOLT = 5.0;
const int   ADC_RESOLUTION = 1023;
const int   PH_SAMPLES = 10;

// ------------------ Water Level Sensor ------------------
#define ATTINY1_HIGH_ADDR   0x78
#define ATTINY2_LOW_ADDR    0x77
#define THRESHOLD           100    // adjust as needed
uint8_t low_data_watersensor[8] = {0};
uint8_t high_data_watersensor[12] = {0};

// ------------------ LoRaWAN ------------------
static const PROGMEM u1_t NWKSKEY[16] = { 0xF0,0xE8,0x88,0xE6,0xF9,0xAD,0x52,0xEE,0x98,0x40,0xFF,0xBA,0xC8,0xD2,0x81,0x5D };
static const PROGMEM u1_t APPSKEY[16] = { 0xE3,0xD0,0xA3,0xEA,0x86,0x14,0xD5,0x3C,0x2B,0x21,0xDF,0x9E,0x89,0x8A,0xFE,0x78 };
static const u4_t DEVADDR = 0x260B48AB;

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

// Payload: 3 sensors x 3 bytes each
uint8_t mydata[9];
const uint8_t TEMP_SENSOR_ID  = 1;
const uint8_t PH_SENSOR_ID    = 2;
const uint8_t WATER_SENSOR_ID = 3;
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

// ------------------ Water Sensor Functions ------------------
bool getHigh12SectionValue() {
    memset(high_data_watersensor, 0, sizeof(high_data_watersensor));
    Wire.requestFrom(ATTINY1_HIGH_ADDR, 12);
    unsigned long start = millis();
    while (Wire.available() < 12) {
        if (millis() - start > 100) {
            Serial.println(F("⚠️ No response from high section (0x78)"));
            return false;
        }
    }
    for (int i = 0; i < 12; i++) high_data_watersensor[i] = Wire.read();
    delay(10);
    return true;
}

bool getLow8SectionValue() {
    memset(low_data_watersensor, 0, sizeof(low_data_watersensor));
    Wire.requestFrom(ATTINY2_LOW_ADDR, 8);
    unsigned long start = millis();
    while (Wire.available() < 8) {
        if (millis() - start > 100) {
            Serial.println(F("⚠️ No response from low section (0x77)"));
            return false;
        }
    }
    for (int i = 0; i < 8; i++) low_data_watersensor[i] = Wire.read();
    delay(10);
    return true;
}

uint16_t readWaterLevel_mm() {
    if (!getLow8SectionValue() || !getHigh12SectionValue()) {
        Serial.println(F("⚠️ Water level sensor read failed"));
        return 0;
    }

    uint8_t wet_sections = 0;
    for (int i = 0; i < 8; i++) if (low_data_watersensor[i] > THRESHOLD) wet_sections++;
    for (int i = 0; i < 12; i++) if (high_data_watersensor[i] > THRESHOLD) wet_sections++;

    Serial.print(F("Wet sections: "));
    Serial.println(wet_sections);

    return wet_sections * 5; // 5 mm per section
}

// ------------------ LoRa Event ------------------
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

// ------------------ Send Data ------------------
void do_send(osjob_t* j) {
    sensor1.requestTemperatures();
    float temp = sensor1.getTempCByIndex(0);
    Serial.print("Sensor 1 temperatuur: ");
    Serial.println(temp);

    // pH calculation
    long phSum = 0;
    for (int i = 0; i < PH_SAMPLES; i++) {
        phSum += analogRead(PH_PIN);
        delay(10);
    }
    float phRaw = (float)phSum / PH_SAMPLES;
    float phVoltage = (phRaw / ADC_RESOLUTION) * ADC_REF_VOLT;
    float phSlope  = (PH_CAL_PH2 - PH_CAL_PH1) / (PH_CAL_V2 - PH_CAL_V1);
    float phOffset = PH_CAL_PH1 - phSlope * PH_CAL_V1;
    float ph = phSlope * phVoltage + phOffset;
    Serial.print("pH spanning (V): ");
    Serial.println(phVoltage, 3);
    Serial.print("Berekenede pH: ");
    Serial.println(ph, 2);

    // Water Level
    uint16_t waterLevel = readWaterLevel_mm();
    Serial.print("Water level (mm): ");
    Serial.println(waterLevel);

    int16_t tempVal  = round(temp * 100);
    int16_t phVal    = round(ph * 100);
    int16_t waterVal = round(waterLevel * 100);

    // Fill payload
    mydata[0] = TEMP_SENSOR_ID;
    mydata[1] = highByte(tempVal);
    mydata[2] = lowByte(tempVal);

    mydata[3] = PH_SENSOR_ID;
    mydata[4] = highByte(phVal);
    mydata[5] = lowByte(phVal);

    mydata[6] = WATER_SENSOR_ID;
    mydata[7] = highByte(waterVal);
    mydata[8] = lowByte(waterVal);

    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
}

// ------------------ Setup ------------------
void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(500); // allow sensor ATtiny to boot

    Serial.println(F("Opstarten met sensors..."));
    sensor1.begin();
    sensor1.setResolution(12);
    sensor1.setWaitForConversion(true);

    // LoRa init
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

    // First send scheduled after setup
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
}

// ------------------ Loop ------------------
void loop() {
    os_runloop_once();
}

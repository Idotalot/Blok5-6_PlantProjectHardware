#pragma once

#include <Arduino.h>
#include <Wire.h>

class WaterLevelSensor {
public:
  WaterLevelSensor(uint8_t highAddress, uint8_t lowAddress, uint8_t threshold)
    : highAddr(highAddress), lowAddr(lowAddress), thresholdValue(threshold) {
    memset(lowData, 0, sizeof(lowData));
    memset(highData, 0, sizeof(highData));
  }

  // Returns water level in millimeters; 5 mm per wet section. 0 on failure.
  uint16_t readWaterLevelMm() {
    if (!readLow8() || !readHigh12()) {
      return 0;
    }
    uint8_t wetSections = 0;
    for (int i = 0; i < 8; i++) if (lowData[i] > thresholdValue) wetSections++;
    for (int i = 0; i < 12; i++) if (highData[i] > thresholdValue) wetSections++;
    return static_cast<uint16_t>(wetSections) * 5; // 5 mm per section
  }

  // Optionally expose the raw wet section count
  uint8_t readWetSectionCount() {
    if (!readLow8() || !readHigh12()) {
      return 0;
    }
    uint8_t wetSections = 0;
    for (int i = 0; i < 8; i++) if (lowData[i] > thresholdValue) wetSections++;
    for (int i = 0; i < 12; i++) if (highData[i] > thresholdValue) wetSections++;
    return wetSections;
  }

private:
  bool readHigh12() {
    memset(highData, 0, sizeof(highData));
    Wire.requestFrom(highAddr, 12);
    unsigned long start = millis();
    while (Wire.available() < 12) {
      if (millis() - start > 100) {
        return false;
      }
    }
    for (int i = 0; i < 12; i++) highData[i] = Wire.read();
    // short non-blocking wait (~10ms)
    unsigned long waitStart = millis();
    while (millis() - waitStart < 10) {}
    return true;
  }

  bool readLow8() {
    memset(lowData, 0, sizeof(lowData));
    Wire.requestFrom(lowAddr, 8);
    unsigned long start = millis();
    while (Wire.available() < 8) {
      if (millis() - start > 100) {
        return false;
      }
    }
    for (int i = 0; i < 8; i++) lowData[i] = Wire.read();
    // short non-blocking wait (~10ms)
    unsigned long waitStart = millis();
    while (millis() - waitStart < 10) {}
    return true;
  }

  uint8_t highAddr;
  uint8_t lowAddr;
  uint8_t thresholdValue;
  uint8_t lowData[8];
  uint8_t highData[12];
};



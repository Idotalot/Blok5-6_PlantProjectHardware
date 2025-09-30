#pragma once

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class TemperatureSensor {
public:
  explicit TemperatureSensor(uint8_t oneWirePin)
    : oneWire(oneWirePin), dallas(&oneWire), waitForConversion(true), resolutionBits(12) {}

  void begin() {
    dallas.begin();
    dallas.setResolution(resolutionBits);
    dallas.setWaitForConversion(waitForConversion);
  }

  void setResolution(uint8_t bits) {
    resolutionBits = bits;
    dallas.setResolution(bits);
  }

  void setWaitForConversion(bool wait) {
    waitForConversion = wait;
    dallas.setWaitForConversion(waitForConversion);
  }

  float readCelsius() {
    dallas.requestTemperatures();
    return dallas.getTempCByIndex(0);
  }

private:
  OneWire oneWire;
  DallasTemperature dallas;
  bool waitForConversion;
  uint8_t resolutionBits;
};



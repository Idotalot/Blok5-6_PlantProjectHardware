#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <stdint.h>
// Minimal forward declarations to satisfy non-Arduino linters
unsigned long millis();
int analogRead(uint8_t pin);
#endif

class PhSensor {
public:
  PhSensor(uint8_t analogPin, float adcRefVolt, int adcResolution, int samples)
    : pin(analogPin), refVolt(adcRefVolt), resolution(adcResolution), numSamples(samples) {}

  void setCalibration(float ph1, float v1, float ph2, float v2) {
    calPh1 = ph1; calV1 = v1; calPh2 = ph2; calV2 = v2;
  }

  // Simple millis-based sampling without delay(). Blocks until samples are collected.
  float readVoltage() const {
    long sumBlocking = 0;
    int collected = 0;
    unsigned long nextAt = 0; // take first sample immediately
    while (collected < numSamples) {
      unsigned long now = millis();
      if (nextAt == 0 || now >= nextAt) {
        sumBlocking += analogRead(pin);
        collected++;
        nextAt = now + sampleIntervalMs;
      }
      // busy-wait; no delay() used to keep timing simple and cooperative
    }
    float raw = (float)sumBlocking / collected;
    return (raw / (float)resolution) * refVolt;
  }

  void setSamplingIntervalMs(uint16_t intervalMs) { sampleIntervalMs = intervalMs; }

  float readPh() const {
    float v = readVoltage();
    float slope = (calPh2 - calPh1) / (calV2 - calV1);
    float offset = calPh1 - slope * calV1;
    return slope * v + offset;
  }

  float phFromVoltage(float v) const {
    float slope = (calPh2 - calPh1) / (calV2 - calV1);
    float offset = calPh1 - slope * calV1;
    return slope * v + offset;
  }

private:
  uint8_t pin;
  float refVolt;
  int resolution;
  int numSamples;
  uint16_t sampleIntervalMs = 10; // default interval between samples
  float calPh1 = 7.0f;
  float calV1  = 1.882f;
  float calPh2 = 4.0f;
  float calV2  = 1.982f;
};



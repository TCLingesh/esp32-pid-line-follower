#pragma once
#include "config.h"

uint16_t sensorValues[SensorCount]    = {1, 1, 1, 1, 1, 1, 1, 1};
uint16_t rawSensorValues[SensorCount] = {0, 0, 0, 0, 0, 0, 0, 0};

float lineError        = 0.0f;
uint8_t noLineCount    = 0;
bool lineLost          = false;
bool centerActive      = false;
bool sharpCornerLeft   = false;
bool sharpCornerRight  = false;

void initSensors() {
  pinMode(IR_ENABLE_PIN, OUTPUT);
  digitalWrite(IR_ENABLE_PIN, HIGH);
}

void readAndCalculateError() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    int raw = analogRead(sensorPins[i]);
    rawSensorValues[i] = (uint16_t)raw;
    sensorValues[i] = (raw > RAW_LINE_THRESHOLD) ? 1000 : 0;
  }

  if (!LINE_READS_HIGH) {
    for (uint8_t i = 0; i < SensorCount; i++) {
      sensorValues[i] = 1000 - sensorValues[i];
    }
  }

  bool lineExists   = false;
  float weightedSum = 0.0f;
  float valueSum    = 0.0f;

  for (uint8_t i = 0; i < SensorCount; i++) {
    float v = sensorValues[i];
    weightedSum += (i - 3.5f) * v; // Symmetric weights: -3.5 to +3.5
    valueSum += v;
    if (sensorValues[i] > 0) {
      lineExists = true;
    }
  }

  bool leftEdgeActive  = (sensorValues[0] > 0 || sensorValues[1] > 0);
  bool rightEdgeActive = (sensorValues[6] > 0 || sensorValues[7] > 0);

  centerActive = (sensorValues[2] > 0 || sensorValues[3] > 0 || 
                  sensorValues[4] > 0 || sensorValues[5] > 0);

  // Trigger sharp turn immediately when outer sensors detect a perpendicular branch
  sharpCornerLeft  = leftEdgeActive && !rightEdgeActive;
  sharpCornerRight = rightEdgeActive && !leftEdgeActive;

  if (lineLost) {
    if (centerActive) {
      lineLost = false;
      noLineCount = 0;
    }
  } else if (lineExists && valueSum > 0) {
    lineError = weightedSum / valueSum;
    noLineCount = 0;
    lineLost = false;
  } else {
    if (noLineCount < 255) noLineCount++;
    if (noLineCount > NO_LINE_COUNT) lineLost = true;
  }
}
#pragma once
#include "config.h"

float pidOutput             = 0.0f;
float pidIntegral           = 0.0f;
float pidLastError          = 0.0f;
float pidFilteredDerivative = 0.0f;
bool  pidFirstRun           = true;

void resetPID() {
  pidIntegral           = 0.0f;
  pidLastError          = 0.0f;
  pidFilteredDerivative = 0.0f;
  pidFirstRun           = true;
}

float computePID(float error) {
  float pTerm = Kp * error;

  pidIntegral += error * CONTROL_PERIOD_S;
  float iTerm = Ki * pidIntegral;

  float rawDerivative = 0.0f;
  if (!pidFirstRun) {
    rawDerivative = (error - pidLastError) / CONTROL_PERIOD_S;
  }

  // Low-pass IIR filter to suppress high-frequency ADC noise in derivative term
  pidFilteredDerivative = D_FILTER_ALPHA * pidFilteredDerivative
                        + (1.0f - D_FILTER_ALPHA) * rawDerivative;
  float dTerm = Kd * pidFilteredDerivative;

  pidLastError = error;
  pidFirstRun  = false;

  float output = pTerm + iTerm + dTerm;

  // Dynamic clamping with integrator anti-windup
  float outMin = PID_OUT_MIN * BASE_SPEED;
  float outMax = PID_OUT_MAX * BASE_SPEED;

  if (output > outMax) {
    output = outMax;
    if (error > 0) pidIntegral -= error * CONTROL_PERIOD_S;
  } else if (output < outMin) {
    output = outMin;
    if (error < 0) pidIntegral -= error * CONTROL_PERIOD_S;
  }

  return output;
}
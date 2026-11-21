#include <Arduino.h>
#include "config.h"
#include "ir_sensor.h"
#include "pid_controller.h"
#include "motor_driver.h"

unsigned long lastControlTime = 0;
unsigned long lastPrintTime   = 0;

void printStatus() {
  Serial.print("Pos:");   Serial.print(lineError);
  Serial.print("\tOut:");   Serial.print(pidOutput);
  Serial.print("\tL:");     Serial.print(leftMotorSpeed);
  Serial.print("\tR:");     Serial.print(rightMotorSpeed);
  Serial.print("\tTurnL:"); Serial.print(isTurningLeft);
  Serial.print("\tTurnR:"); Serial.print(isTurningRight);
  Serial.print("\tLost:");  Serial.print(lineLost);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  initMotors();
  initSensors();
  resetPID();
}

void loop() {
  unsigned long now = millis();

  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    lastControlTime = now;
    readAndCalculateError();
    calculateMotorSpeed();
    setMotorSpeed();
  }

  if (now - lastPrintTime >= PRINT_PERIOD_MS) {
    lastPrintTime = now;
    printStatus();
  }
}
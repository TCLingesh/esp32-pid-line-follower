#pragma once
#include "config.h"
#include "ir_sensor.h"
#include "pid_controller.h"

int leftMotorSpeed  = 0;
int rightMotorSpeed = 0;

bool isTurningLeft  = false;
bool isTurningRight = false;
bool lineLostActive = false;

unsigned long turnStartTime     = 0;
unsigned long lastTurnEndTime   = 0;
unsigned long lineLostStartTime = 0;

void initMotors() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

void driveMotor(int in1Pin, int in2Pin, int pwmPin, int speed) {
  if (speed >= 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    speed = -speed;
  }

  speed = constrain(speed, 0, 255);
  analogWrite(pwmPin, speed);
}

void setMotorSpeed() {
  driveMotor(AIN1, AIN2, PWMA, leftMotorSpeed);
  driveMotor(BIN1, BIN2, PWMB, rightMotorSpeed);
}

void calculateMotorSpeed() {
  // --- 1. LATCHED 90-DEGREE SHARP CORNER OVERRIDE ---
  if (millis() - lastTurnEndTime > TURN_COOLDOWN_MS) {
    if (sharpCornerLeft && !isTurningLeft && !isTurningRight) {
      leftMotorSpeed  = 0;
      rightMotorSpeed = 0;
      setMotorSpeed();
      delay(12); // Brief active brake to kill forward inertia

      isTurningLeft = true;
      turnStartTime = millis();
    }
    if (sharpCornerRight && !isTurningLeft && !isTurningRight) {
      leftMotorSpeed  = 0;
      rightMotorSpeed = 0;
      setMotorSpeed();
      delay(12);

      isTurningRight = true;
      turnStartTime = millis();
    }
  }

  // Execute zero-radius differential pivot turn
  if (isTurningLeft || isTurningRight) {
    if (isTurningLeft) {
      leftMotorSpeed  = -TURN_BOOST_SPEED;
      rightMotorSpeed =  TURN_BOOST_SPEED;
    } else {
      leftMotorSpeed  =  TURN_BOOST_SPEED;
      rightMotorSpeed = -TURN_BOOST_SPEED;
    }

    bool innerSensorsActive = (sensorValues[2] > 0 || sensorValues[3] > 0 ||
                               sensorValues[4] > 0 || sensorValues[5] > 0);
    bool timedOut = (millis() - turnStartTime > MAX_TURN_DURATION_MS);

    if (innerSensorsActive || timedOut) {
      isTurningLeft   = false;
      isTurningRight  = false;
      lastTurnEndTime = millis();
      lineError       = 0;

      // Active counter-brake to prevent line overshoot
      leftMotorSpeed  = 0;
      rightMotorSpeed = 0;
      setMotorSpeed();

      if (timedOut && !centerActive) {
        lineLost = true;
      }
    }

    resetPID();
    if (!lineLost) return;
  }

  // --- 2. APEX LINE-LOSS RECOVERY ---
  if (lineLost) {
    if (millis() - lastTurnEndTime < TURN_COOLDOWN_MS) {
      leftMotorSpeed  = -90;
      rightMotorSpeed = -90;
      return;
    }

    if (!lineLostActive) {
      lineLostStartTime = millis();
      lineLostActive = true;
    }

    if (millis() - lineLostStartTime < REVERSE_TIME_MS) {
      leftMotorSpeed  = -100;
      rightMotorSpeed = -100;
    } else {
      // Spin toward the last known line error polarity
      if (lineError < 0) {
        leftMotorSpeed  = -TURN_BOOST_SPEED;
        rightMotorSpeed =  TURN_BOOST_SPEED;
      } else {
        leftMotorSpeed  =  TURN_BOOST_SPEED;
        rightMotorSpeed = -TURN_BOOST_SPEED;
      }
    }

    resetPID();
    return;
  } else {
    lineLostActive = false;
  }

  // --- 3. STANDARD 500 Hz PID TRACKING ---
  pidOutput = computePID(lineError);

  leftMotorSpeed  = constrain(BASE_SPEED + (int)pidOutput, -MAX_SPEED, MAX_SPEED);
  rightMotorSpeed = constrain(BASE_SPEED - (int)pidOutput, -MAX_SPEED, MAX_SPEED);
}
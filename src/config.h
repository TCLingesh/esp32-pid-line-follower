#pragma once
#include <Arduino.h>

// =====================================================
// MOTOR DRIVER PINS (TB6612FNG) -- ESP32 Pinout
// =====================================================
#define AIN1 17
#define AIN2 18
#define PWMA 16

#define BIN1 21
#define BIN2 13
#define PWMB 19

#define STBY 22

// =====================================================
// 8-CHANNEL IR SENSOR ARRAY PINS
// =====================================================
#define IR_ENABLE_PIN 23
const uint8_t SensorCount = 8;
const uint8_t sensorPins[SensorCount] = {14, 27, 26, 25, 33, 32, 35, 34}; // Reversed physical order

// =====================================================
// SENSOR THRESHOLDING
// =====================================================
const uint16_t RAW_LINE_THRESHOLD = 3700; // 12-bit ESP32 ADC (0 - 4095)
const bool LINE_READS_HIGH = true;        // true = dark line on white surface

// =====================================================
// PID & SPEED TUNING PARAMETERS
// =====================================================
float Kp = 90.0f;
float Ki = 1.0f;
float Kd = 153.0f;

int BASE_SPEED       = 190; // Nominal forward PWM (0 - 255)
int MAX_SPEED        = 230; // Upper PWM saturation limit
int TURN_BOOST_SPEED = 220; // Differential pivot speed for 90-degree corners

// =====================================================
// CONTROL LOOP & RECOVERY TIMINGS
// =====================================================
const unsigned long CONTROL_PERIOD_MS    = 2;     // 500 Hz control loop
const float CONTROL_PERIOD_S             = CONTROL_PERIOD_MS / 1000.0f;
const unsigned long PRINT_PERIOD_MS      = 100;   // 10 Hz serial telemetry

const uint8_t NO_LINE_COUNT              = 8;     // Consecutive blank reads before lineLost triggers
const unsigned long REVERSE_TIME_MS      = 60;    // Reverse brake pulse on line loss
const unsigned long MAX_TURN_DURATION_MS = 350;   // Failsafe timeout for latched 90-deg turns
const unsigned long TURN_COOLDOWN_MS     = 80;    // Debounce cooldown between sharp corners

const float D_FILTER_ALPHA               = 0.6f;  // Low-pass filter weight for derivative term
const float PID_OUT_MIN                  = -1.5f; // Lower anti-windup output multiplier
const float PID_OUT_MAX                  =  1.5f; // Upper anti-windup output multiplier
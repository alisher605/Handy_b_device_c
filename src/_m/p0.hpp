#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <BleMouse.h>
#include <MPU6050_tockn.h>

#define PIN_SDA 21
#define PIN_SCL 18

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

static const uint16_t FSR_LIGHT_THRESHOLD = 650;
static const uint16_t FSR_HARD_THRESHOLD  = 2700;

#define FSR_PIN 34
#define FSR_ACTIVE_LOW 0

static uint16_t fsrBaseline = 0;
static uint16_t fsrLightThreshold = 0;
static uint16_t fsrHardThreshold  = 0;

static void updateFsrThresholds() {
  fsrLightThreshold = FSR_LIGHT_THRESHOLD;
  fsrHardThreshold  = FSR_HARD_THRESHOLD;
}

static const uint16_t FSR_HYST = 120;
static const float EMA_ALPHA = 0.25f;

static const uint32_t PRESS_CONFIRM_MS   = 50;
static const uint32_t RELEASE_CONFIRM_MS = 60;
static const uint32_t CLICK_LOCKOUT_MS   = 250;
static const uint16_t FSR_MIN_VALID = 0;
static const uint16_t FSR_MAX_VALID = 4095;
static const uint8_t FSR_VALID_STREAK_FOR_PRESS = 1;
static const uint8_t FSR_INVALID_STREAK_FOR_RELEASE = 1;

static const float DEADZONE_DEG = 1.8f;
static const float MAX_TILT_DEG = 30.0f;
static const int   MAX_SPEED    = 60;
static const int   UPDATE_HZ    = 40;
static const float SMOOTH_ALPHA = 0.15f;

static const int INVERT_X = -1;
static const int INVERT_Y = 1;
static const bool  SWAP_AXES     = true;
static const float MOUNT_ROT_DEG = 0.0f;

static const uint32_t CLICK_DEBOUNCE_MS = 90;
static const uint32_t LONG_PRESS_MS     = 500;
static const uint32_t DOUBLE_TAP_MS     = 300;

static const uint32_t MPU_WARMUP_MS              = 1200;
static const int      MPU_ZERO_SAMPLES           = 120;
static const uint32_t MPU_ZERO_SAMPLE_DELAY_MS   = 8;
static const float    MPU_STILL_RANGE_DEG        = 1.2f;
static const int      MPU_ZERO_ATTEMPTS          = 3;
static const bool     MPU_REQUIRE_STILLNESS      = false;

static BleMouse bleMouse("HANDY Glove", "HANDY", 100);
static MPU6050 mpu(Wire);

static float smoothRoll  = 0.0f;
static float smoothPitch = 0.0f;

static float rollZero  = 0.0f;
static float pitchZero = 0.0f;

static uint8_t fsrLevel = 0;
static float fsrFiltered = 0.0f;

static uint8_t pendingLevel = 0;
static uint32_t pendingSince = 0;
static uint32_t lastClickAt = 0;

static bool mouseButtonDown = false;
static uint32_t fsrDownAt = 0;
static uint32_t lastFsrChangeAt = 0;
static uint16_t fsrLastValue = 0;
static uint8_t fsrValidStreak = 0;
static uint8_t fsrInvalidStreak = 0;
static bool waitingSecondTap = false;
static uint32_t lastTapReleaseAt = 0;

static uint32_t lastUpdateAt = 0;

static int tiltToSpeed(float tiltDeg);
static void setLed(bool on);
static void warmupMpu();
static void calibrateMpuZero();
static void printSensorDebug(float roll, float pitch, int dx, int dy);

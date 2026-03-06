#pragma once

#include "p0.hpp"

static int tiltToSpeed(float tiltDeg) {
  float absTilt = fabsf(tiltDeg);
  if (absTilt < DEADZONE_DEG) return 0;

  float sign = (tiltDeg >= 0) ? 1.0f : -1.0f;
  float t = (absTilt - DEADZONE_DEG) / (MAX_TILT_DEG - DEADZONE_DEG);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  int speed = (int)(t * MAX_SPEED);
  if (speed < 1) speed = 1;
  return (int)(sign * speed);
}

static void setLed(bool on) {
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

static void warmupMpu() {
  uint32_t start = millis();
  while (millis() - start < MPU_WARMUP_MS) {
    mpu.update();
    delay(5);
  }
}

static void calibrateMpuZero() {
  Serial.println("Calibrating MPU6050 neutral (zero)... keep device in neutral position");
  warmupMpu();

  float lastSumRoll = 0.0f;
  float lastSumPitch = 0.0f;

  int attempts = MPU_REQUIRE_STILLNESS ? MPU_ZERO_ATTEMPTS : 1;
  for (int attempt = 1; attempt <= attempts; attempt++) {
    float sumRoll = 0.0f;
    float sumPitch = 0.0f;
    float minRoll = 1e9f, maxRoll = -1e9f;
    float minPitch = 1e9f, maxPitch = -1e9f;

    for (int i = 0; i < MPU_ZERO_SAMPLES; i++) {
      mpu.update();

      float r = mpu.getAngleY();
      float p = mpu.getAngleX();

      sumRoll  += r;
      sumPitch += p;

      if (r < minRoll) minRoll = r;
      if (r > maxRoll) maxRoll = r;
      if (p < minPitch) minPitch = p;
      if (p > maxPitch) maxPitch = p;

      delay(MPU_ZERO_SAMPLE_DELAY_MS);
    }

    lastSumRoll = sumRoll;
    lastSumPitch = sumPitch;

    float rollRange = maxRoll - minRoll;
    float pitchRange = maxPitch - minPitch;

    if (!MPU_REQUIRE_STILLNESS) {
      rollZero  = sumRoll / (float)MPU_ZERO_SAMPLES;
      pitchZero = sumPitch / (float)MPU_ZERO_SAMPLES;

      Serial.print("Zero set (moving OK). rollZero=");
      Serial.print(rollZero, 2);
      Serial.print(" pitchZero=");
      Serial.println(pitchZero, 2);
      return;
    }

    if (rollRange <= MPU_STILL_RANGE_DEG && pitchRange <= MPU_STILL_RANGE_DEG) {
      rollZero  = sumRoll / (float)MPU_ZERO_SAMPLES;
      pitchZero = sumPitch / (float)MPU_ZERO_SAMPLES;

      Serial.print("Zero set. rollZero=");
      Serial.print(rollZero, 2);
      Serial.print(" pitchZero=");
      Serial.println(pitchZero, 2);
      return;
    }

    Serial.print("Zero not stable (range roll=");
    Serial.print(rollRange, 2);
    Serial.print(" pitch=");
    Serial.print(pitchRange, 2);
    Serial.println("). Hold still...");
    delay(600);
    warmupMpu();
  }

  rollZero  = lastSumRoll / (float)MPU_ZERO_SAMPLES;
  pitchZero = lastSumPitch / (float)MPU_ZERO_SAMPLES;

  Serial.print("Zero set (fallback). rollZero=");
  Serial.print(rollZero, 2);
  Serial.print(" pitchZero=");
  Serial.println(pitchZero, 2);
}

static void printSensorDebug(float roll, float pitch, int dx, int dy) {
  static uint32_t lastPrint = 0;
  const uint32_t PRINT_INTERVAL_MS = 200;

  uint32_t now = millis();
  if (now - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = now;

  Serial.print("MPU | Roll(deg): ");  Serial.print(roll, 2);
  Serial.print(" | Pitch(deg): ");   Serial.print(pitch, 2);
  Serial.print(" | dX: ");           Serial.print(dx);
  Serial.print(" | dY: ");           Serial.print(dy);
  Serial.print(" | FSR: ");          Serial.print(fsrLastValue);
  Serial.print("/");                 Serial.print((int)fsrFiltered);
  Serial.print(" | Level: ");        Serial.print(fsrLevel);
  Serial.print(" | Thr L/H: ");      Serial.print(fsrLightThreshold);
  Serial.print("/");                 Serial.println(fsrHardThreshold);
}

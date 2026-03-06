#pragma once

#include "p1.hpp"

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  setLed(false);

  Serial.begin(115200);
  delay(200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  mpu.begin();

  warmupMpu();

  Serial.println("Calibrating gyro offsets... keep device still");
  mpu.calcGyroOffsets(true);

  warmupMpu();

  calibrateMpuZero();

  analogReadResolution(12);
  pinMode(FSR_PIN, INPUT);
  #if defined(ESP_ARDUINO_VERSION_MAJOR)
    analogSetPinAttenuation(FSR_PIN, ADC_11db);
  #endif

  uint16_t v = analogRead(FSR_PIN);
  fsrFiltered = (float)v;

  delay(400);
  uint32_t sum = 0;
  for (int i = 0; i < 60; i++) {
    sum += analogRead(FSR_PIN);
    delay(10);
  }
  fsrBaseline = sum / 60;
  updateFsrThresholds();

  Serial.print("FSR baseline: ");
  Serial.print(fsrBaseline);
#if FSR_ACTIVE_LOW
  Serial.print(" light<=");
  Serial.print(fsrLightThreshold);
  Serial.print(" hard<=");
  Serial.println(fsrHardThreshold);
#else
  Serial.print(" light>=");
  Serial.print(fsrLightThreshold);
  Serial.print(" hard>=");
  Serial.println(fsrHardThreshold);
#endif

  bleMouse.begin();

  Serial.println("HANDY ready. Pair as BLE mouse: 'HANDY Glove'");
}

void loop() {
  mpu.update();

  const uint32_t intervalMs = (uint32_t)(1000 / UPDATE_HZ);
  uint32_t now = millis();

  (void)analogRead(FSR_PIN);

  uint16_t r0 = analogRead(FSR_PIN);
  uint16_t r1 = analogRead(FSR_PIN);
  uint16_t r2 = analogRead(FSR_PIN);
  fsrLastValue = (uint16_t)((r0 + r1 + r2) / 3);

  bool adcValid = (fsrLastValue >= FSR_MIN_VALID && fsrLastValue <= FSR_MAX_VALID);
  if (adcValid) {
    fsrFiltered = EMA_ALPHA * (float)fsrLastValue + (1.0f - EMA_ALPHA) * fsrFiltered;
    if (fsrValidStreak < 255) fsrValidStreak++;
    fsrInvalidStreak = 0;
  } else {
    if (fsrInvalidStreak < 255) fsrInvalidStreak++;
    fsrValidStreak = 0;
  }

  uint8_t desiredLevel = fsrLevel;
  if (!adcValid) {
    desiredLevel = (fsrInvalidStreak >= FSR_INVALID_STREAK_FOR_RELEASE) ? 0 : fsrLevel;
  } else {
#if FSR_ACTIVE_LOW
    if (fsrLevel == 0) {
      if (fsrFiltered <= (float)fsrHardThreshold) desiredLevel = 2;
      else if (fsrFiltered <= (float)fsrLightThreshold) desiredLevel = 1;
    } else if (fsrLevel == 1) {
      if (fsrFiltered <= (float)fsrHardThreshold) desiredLevel = 2;
      else if (fsrFiltered > (float)(fsrLightThreshold + FSR_HYST)) desiredLevel = 0;
    } else {
      if (fsrFiltered > (float)(fsrHardThreshold + FSR_HYST)) {
        desiredLevel = (fsrFiltered <= (float)fsrLightThreshold) ? 1 : 0;
      }
    }
#else
    if (fsrLevel == 0) {
      if (fsrFiltered >= (float)fsrHardThreshold) desiredLevel = 2;
      else if (fsrFiltered >= (float)fsrLightThreshold) desiredLevel = 1;
    } else if (fsrLevel == 1) {
      if (fsrFiltered >= (float)fsrHardThreshold) desiredLevel = 2;
      else if (fsrFiltered < (float)(fsrLightThreshold - FSR_HYST)) desiredLevel = 0;
    } else {
      if (fsrFiltered < (float)(fsrHardThreshold - FSR_HYST)) {
        desiredLevel = (fsrFiltered >= (float)fsrLightThreshold) ? 1 : 0;
      }
    }
#endif
    if (fsrLevel == 0 && desiredLevel > 0 && fsrValidStreak < FSR_VALID_STREAK_FOR_PRESS) {
      desiredLevel = 0;
    }
  }

  if (desiredLevel != fsrLevel) {
    if (pendingLevel != desiredLevel) {
      pendingLevel = desiredLevel;
      pendingSince = now;
    }

    uint32_t confirmMs = (pendingLevel == 0) ? RELEASE_CONFIRM_MS : PRESS_CONFIRM_MS;
    if ((now - pendingSince) >= confirmMs && (now - lastFsrChangeAt) > CLICK_DEBOUNCE_MS) {
      uint8_t prevLevel = fsrLevel;
      fsrLevel = pendingLevel;
      lastFsrChangeAt = now;

      if (prevLevel == 0 && fsrLevel > 0) {
        fsrDownAt = now;
      }

      if (prevLevel > 0 && fsrLevel == 0) {
        uint32_t held = now - fsrDownAt;

        if (bleMouse.isConnected() && mouseButtonDown) {
          bleMouse.release(MOUSE_LEFT);
          mouseButtonDown = false;
          waitingSecondTap = false;
        } else if (held < LONG_PRESS_MS) {
          if (bleMouse.isConnected() && (now - lastClickAt) > CLICK_LOCKOUT_MS) {
            if (waitingSecondTap && (now - lastTapReleaseAt) <= DOUBLE_TAP_MS) {
              bleMouse.click(MOUSE_RIGHT);
              lastClickAt = now;
              waitingSecondTap = false;
            } else {
              waitingSecondTap = true;
              lastTapReleaseAt = now;
            }
          }
        } else {
          waitingSecondTap = false;
        }
      }

      pendingLevel = fsrLevel;
      pendingSince = now;
    }
  } else {
    pendingLevel = fsrLevel;
    pendingSince = now;
  }

  if (waitingSecondTap && fsrLevel == 0 && (now - lastTapReleaseAt) > DOUBLE_TAP_MS) {
    if (bleMouse.isConnected() && (now - lastClickAt) > CLICK_LOCKOUT_MS) {
      bleMouse.click(MOUSE_LEFT);
      lastClickAt = now;
    }
    waitingSecondTap = false;
  }

  if (fsrLevel > 0 && bleMouse.isConnected()) {
    uint32_t held = now - fsrDownAt;
    if (!mouseButtonDown && held >= LONG_PRESS_MS) {
      bleMouse.press(MOUSE_LEFT);
      mouseButtonDown = true;
      waitingSecondTap = false;
    }
  }

  if (now - lastUpdateAt >= intervalMs) {
    lastUpdateAt = now;

    float roll  = (mpu.getAngleY() - rollZero);
    float pitch = (mpu.getAngleX() - pitchZero);

    if (SWAP_AXES) {
      float tmp = roll;
      roll = pitch;
      pitch = tmp;
    }

    if (MOUNT_ROT_DEG != 0.0f) {
      float rad = MOUNT_ROT_DEG * (PI / 180.0f);
      float c = cosf(rad);
      float s = sinf(rad);
      float r = (roll * c) - (pitch * s);
      float p = (roll * s) + (pitch * c);
      roll = r;
      pitch = p;
    }

    roll  *= (float)INVERT_X;
    pitch *= (float)INVERT_Y;

    smoothRoll  = (SMOOTH_ALPHA * roll)  + ((1.0f - SMOOTH_ALPHA) * smoothRoll);
    smoothPitch = (SMOOTH_ALPHA * pitch) + ((1.0f - SMOOTH_ALPHA) * smoothPitch);

    int dx = tiltToSpeed(smoothRoll);
    int dy = tiltToSpeed(smoothPitch);

    const int MAX_STEP = 10;
    if (dx >  MAX_STEP) dx =  MAX_STEP;
    if (dx < -MAX_STEP) dx = -MAX_STEP;
    if (dy >  MAX_STEP) dy =  MAX_STEP;
    if (dy < -MAX_STEP) dy = -MAX_STEP;

    printSensorDebug(smoothRoll, smoothPitch, dx, dy);

    setLed(bleMouse.isConnected());

    if (bleMouse.isConnected()) {
      if (abs(dx) + abs(dy) >= 2) {
        static uint32_t lastBleSend = 0;
        if (millis() - lastBleSend >= 8) {
          bleMouse.move(dx, dy, 0);
          lastBleSend = millis();
        }
      }
    }
  }
}

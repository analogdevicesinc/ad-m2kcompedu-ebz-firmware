/*
 * ad-m2kcompedu-ebz-firmware.ino - Main sketch
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "src/drivers/Display.h"
#include "src/drivers/Encoder.h"
#include "src/drivers/LedMatrix.h"
#include "src/drivers/AD5443.h"
#include "src/drivers/AD5625R.h"
#include "src/app/Controler.h"
#include "src/app/Peripherals.h"
#include "src/iio/iio_server.h"

// --- Pin definitions ---
#define PIN_ENCODER_CLK   A1
#define PIN_ENCODER_DT    A0
#define PIN_ENCODER_SW    0

#define PIN_AD9833_CS     10
#define PIN_AD5443_SYNC   8
#define PIN_SQW_SWITCH    6
#define PIN_PWM           9

#define ADDR_DISPLAY      0x3C
#define ADDR_AD5625R      0x1E

#define LOOP_PERIOD_MS 1UL

// --- Driver objects ---
Display   display(ADDR_DISPLAY, 64, 128);
Encoder   encoder(PIN_ENCODER_CLK, PIN_ENCODER_DT, PIN_ENCODER_SW);
LedMatrix ledMatrix;

AD9833  waveformGen(PIN_AD9833_CS);
AD5443  amplitudeCtrl(PIN_AD5443_SYNC);
AD5625R dcOffsetCtrl(ADDR_AD5625R);
PwmOut  pwmOut(PIN_PWM);

// --- App objects ---
Peripherals peripherals(waveformGen, amplitudeCtrl, dcOffsetCtrl, pwmOut, PIN_SQW_SWITCH);
Controler controller(encoder, display, peripherals);

// --- Timmers ---
unsigned long currentTimestamp;
unsigned long lastTimestamp;

void setup() {
    Serial.begin(57600);
    controller.begin();
    ledMatrix.begin();

    lastTimestamp = millis();
}

void loop() {
    currentTimestamp = millis();
    if (currentTimestamp - lastTimestamp >= LOOP_PERIOD_MS) {
        controller.tick();
        ledMatrix.tick();
    }
    iio_server_poll();
}

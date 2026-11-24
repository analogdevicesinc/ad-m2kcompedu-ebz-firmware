/*
 * Audio.h
 * Scope: Two-channel ADC capture with timer ISR.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AUDIO_H
#define AUDIO_H

#include <Arduino.h>
#include "FspTimer.h"
#include "Messenger.h"

constexpr uint16_t AUDIO_BUF_SIZE    = 256;
constexpr uint32_t AUDIO_SAMPLE_RATE = 8000;

class Audio {
public:
    Audio(uint8_t preAmpPin, uint8_t postAmpPin, Messenger& messenger);

    void init();
    void run();

private:
    void start();
    void stop();

    Messenger& messenger;
    uint8_t pinPre;
    uint8_t pinPost;
    bool dualChannel;

    volatile int16_t preBuf[2][AUDIO_BUF_SIZE];
    volatile int16_t postBuf[2][AUDIO_BUF_SIZE];
    volatile uint16_t isrIdx;
    volatile uint8_t fill;
    volatile bool ready;

    FspTimer sampleTimer;

    static Audio* instance;
    static void timerCallback(timer_callback_args_t* args);
    void sample();
};

#endif

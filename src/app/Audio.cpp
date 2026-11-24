/*
 * Audio.cpp
 * Scope: Two-channel ADC capture with timer ISR (ping-pong buffering).
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Audio.h"

Audio* Audio::instance = nullptr;

Audio::Audio(uint8_t preAmpPin, uint8_t postAmpPin, Messenger& messenger)
    : messenger(messenger), pinPre(preAmpPin), pinPost(postAmpPin), dualChannel(true),
      isrIdx(0), fill(0), ready(false) {
    instance = this;
}

void Audio::init() {
    analogReadResolution(14);
    pinMode(pinPre, INPUT);
    pinMode(pinPost, INPUT);
    memset((void*)preBuf, 0, sizeof(preBuf));
    memset((void*)postBuf, 0, sizeof(postBuf));
    isrIdx = 0;
    fill = 0;
    ready = false;
    AppState& s = messenger.state();
    s.audioRaw[0] = 0;
    s.audioRaw[1] = 0;
}

void Audio::start() {
    isrIdx = 0;
    fill = 0;
    ready = false;

    uint8_t timerType = GPT_TIMER;
    int8_t channel = FspTimer::get_available_timer(timerType);
    if (channel < 0) return;

    sampleTimer.begin(TIMER_MODE_PERIODIC, timerType, channel,
                      (float)AUDIO_SAMPLE_RATE, 0.0f, timerCallback);
    sampleTimer.setup_overflow_irq();
    sampleTimer.open();
    sampleTimer.start();
}

void Audio::stop() {
    sampleTimer.stop();
    sampleTimer.close();
}

void Audio::run() {
    bool wantActive = messenger.audioActive()
        && messenger.serialAvailable();

    if (wantActive && !sampleTimer.is_opened())
        start();
    else if (!wantActive && sampleTimer.is_opened())
        stop();

    if (!sampleTimer.is_opened() || !ready)
        return;

    __DMB();
    const volatile int16_t* pre = preBuf[1 - fill];
    const volatile int16_t* post = postBuf[1 - fill];

    if (messenger.audioStreaming()
        && messenger.serialUser() == SerialUser::Audio) {
        static const uint8_t sync[2] = {0xAA, 0x55};
        uint8_t channels = dualChannel ? 2 : 1;
        Serial.write(sync, 2);
        Serial.write(channels);
        Serial.write((const uint8_t*)pre, AUDIO_BUF_SIZE * sizeof(int16_t));
        if (dualChannel)
            Serial.write((const uint8_t*)post, AUDIO_BUF_SIZE * sizeof(int16_t));
    }

    AppState& s = messenger.state();
    int32_t sum = 0;
    for (uint16_t i = 0; i < AUDIO_BUF_SIZE; i++)
        sum += pre[i];
    int16_t avg = (int16_t)(sum / AUDIO_BUF_SIZE);
    s.audioRaw[0] = (int16_t)((int32_t)avg * ADC_FULL_SCALE_MV / ADC_MAX_COUNT);

    if (dualChannel) {
        sum = 0;
        for (uint16_t i = 0; i < AUDIO_BUF_SIZE; i++)
            sum += post[i];
        avg = (int16_t)(sum / AUDIO_BUF_SIZE);
        s.audioRaw[1] = (int16_t)((int32_t)avg * ADC_FULL_SCALE_MV / ADC_MAX_COUNT);
    }

    ready = false;
}

void Audio::timerCallback(timer_callback_args_t* args) {
    (void)args;
    if (instance) instance->sample();
}

void Audio::sample() {
    preBuf[fill][isrIdx] = analogRead(pinPre);
    if (dualChannel)
        postBuf[fill][isrIdx] = analogRead(pinPost);
    isrIdx++;

    if (isrIdx < AUDIO_BUF_SIZE)
        return;

    isrIdx = 0;

    if (ready)
        return;

    __DMB();
    fill = 1 - fill;
    ready = true;
}

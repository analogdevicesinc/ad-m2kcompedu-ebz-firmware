/*
 * Peripherals.h
 * Scope: Bridges AppState to hardware drivers.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>
#include <AD9833.h>
#include "pwm.h"
#include "../drivers/AD5443.h"
#include "../drivers/AD5625R.h"

struct AppState;

class Peripherals
{
    public:
        Peripherals(AD9833& waveformGen, AD5443& amplitudeCtrl,
                    AD5625R& dcOffsetCtrl, PwmOut& pwmOut,
                    uint8_t sqwSwitchPin);

        // Signal-gen path: AD9833 (SPI), AD5443 (SPI), AD5625R (I2C).
        // The SPI pins are shared with the CAN transceiver, so the caller
        // must only drive these in SigGen mode.
        void beginSignalGen();
        void updateSignalGen(const AppState& state, bool* refreshFlags);

        // PWM path: independent pin, safe in any mode.
        void beginPwm();
        void updatePwm(const AppState& state, bool* refreshFlags);

    private:
        AD9833&  waveformGen;
        AD5443&  amplitudeCtrl;
        AD5625R& dcOffsetCtrl;
        PwmOut&  pwmOut;
        uint8_t  sqwSwitchPin;
        bool     pwmRunning;

        void updateWave(const AppState& state);
        void updateFrequency(const AppState& state);
        void updateAmplitude(const AppState& state);
        void updateDcOffset(const AppState& state);
        void applyPwm(const AppState& state);
};

#endif

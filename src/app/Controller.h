/*
 * Controller.h
 * Scope: Thin scheduler. Owns the shared state and the app modules, calls
 *        their init()/run() methods.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../drivers/Encoder.h"
#include "../drivers/Display.h"
#include "../drivers/LedMatrix.h"
#include "Messenger.h"
#include "BusArbiter.h"
#include "Menu.h"
#include "CalibrationWizard.h"
#include "Peripherals.h"
#include "CanMessenger.h"
#include "IioAttrServer.h"
#include "AdcCapture.h"

class Controller
{
    public:
        Controller(Encoder& encoder, Display& display,
                   AD9833& waveformGen, AD5443& amplitudeCtrl,
                   AD5625R& dcOffsetCtrl, PwmOut& pwmOut,
                   uint8_t sqwSwitchPin, uint8_t canRxPin,
                   uint8_t adcPreAmpPin, uint8_t adcPostAmpPin);
        void init();
        void tick();

    private:
        Messenger         messenger;
        BusArbiter        arbiter;
        Menu              menu;
        CalibrationWizard wizard;
        Peripherals       peripherals;
        CanMessenger      canMessenger;
        IioAttrServer     iio;
        AdcCapture        adcCapture;
        LedMatrix         ledMatrix;
};

#endif

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
#include "Messenger.h"
#include "BusArbiter.h"
#include "Menu.h"
#include "CalibrationWizard.h"
#include "Peripherals.h"
#include "CanMessenger.h"
#include "IioAttrServer.h"
#include "Audio.h"

class Controller
{
    public:
        Controller(Encoder& encoder, Display& display,
                   AD9833& waveformGen, AD5443& amplitudeCtrl,
                   AD5625R& dcOffsetCtrl, PwmOut& pwmOut,
                   uint8_t sqwSwitchPin, uint8_t canRxPin,
                   uint8_t audioPreAmpPin, uint8_t audioPostAmpPin);
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
        Audio             audio;
};

#endif

/*
 * Controller.cpp
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Controller.h"

Controller::Controller(Encoder& encoder, Display& display,
                       AD9833& waveformGen, AD5443& amplitudeCtrl,
                       AD5625R& dcOffsetCtrl, PwmOut& pwmOut,
                       uint8_t sqwSwitchPin, uint8_t canRxPin,
                       uint8_t audioPreAmpPin, uint8_t audioPostAmpPin)
    : messenger(),
      arbiter(messenger, canRxPin),
      menu(encoder, display, messenger),
      wizard(messenger),
      peripherals(waveformGen, amplitudeCtrl, dcOffsetCtrl, pwmOut, sqwSwitchPin, messenger),
      canMessenger(messenger),
      iio(messenger, menu.paramTable()),
      audio(audioPreAmpPin, audioPostAmpPin, messenger)
{
}

void Controller::init()
{
    arbiter.init();
    menu.init();
    wizard.init();
    peripherals.init();
    canMessenger.init();
    iio.init();
    audio.init();
}

void Controller::tick()
{
    arbiter.run();
    menu.run();
    wizard.run();
    canMessenger.run();
    iio.run();
    audio.run();
    peripherals.run();
}

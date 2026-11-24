/*
 * BusArbiter.h
 * Scope: Owns detection of the shared SPI/CAN jumper (P22) and publishes the
 *        current bus owner to the shared state.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BUS_ARBITER_H
#define BUS_ARBITER_H

#include <Arduino.h>
#include "Messenger.h"

class BusArbiter
{
    public:
        BusArbiter(Messenger& messenger, uint8_t canRxPin);

        void init();
        void run();

    private:
        SpiCanBusOwner detect();

        Messenger& messenger;
        uint8_t    canRxPin;
        uint8_t    lowStreak;
};

#endif

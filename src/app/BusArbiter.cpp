/*
 * BusArbiter.cpp
 * Scope: Owns detection of the shared SPI/CAN jumper (P22) and publishes the
 *        current bus owner to the shared state.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BusArbiter.h"
#include <Arduino.h>

static constexpr uint8_t LOW_STREAK_THRESHOLD = 10;

BusArbiter::BusArbiter(Messenger& messenger, uint8_t canRxPin)
    : messenger(messenger), canRxPin(canRxPin), lowStreak(0)
{
}

SpiCanBusOwner BusArbiter::detect()
{
    pinMode(canRxPin, INPUT);

    SpiCanBusOwner owner = SpiCanBusOwner::SignalGen;
    unsigned long t0 = millis();
    while (millis() - t0 < 10)
    {
        if (digitalRead(canRxPin) == HIGH)
        {
            owner = SpiCanBusOwner::Can;
            break;
        }
    }

    return owner;
}

void BusArbiter::init()
{
    messenger.publishSpiCanBusOwner(detect());
}

void BusArbiter::run()
{
    if (messenger.canActive())
    {
        return;
    }

    pinMode(canRxPin, INPUT);

    SpiCanBusOwner current = messenger.readSpiCanBusOwner();
    if (digitalRead(canRxPin) == HIGH)
    {
        lowStreak = 0;
        if (current != SpiCanBusOwner::Can)
        {
            messenger.publishSpiCanBusOwner(SpiCanBusOwner::Can);
        }
    }
    else
    {
        if (lowStreak < LOW_STREAK_THRESHOLD)
        {
            lowStreak++;
        }
        if (lowStreak >= LOW_STREAK_THRESHOLD && current != SpiCanBusOwner::SignalGen)
        {
            messenger.publishSpiCanBusOwner(SpiCanBusOwner::SignalGen);
        }
    }
}

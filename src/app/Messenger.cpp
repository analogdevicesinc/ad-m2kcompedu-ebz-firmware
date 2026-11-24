/*
 * Messenger.cpp
 * Scope: Single source of truth for state shared between app modules.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Messenger.h"

AppState& Messenger::state()
{
    return appState;
}

const AppState& Messenger::state() const
{
    return appState;
}

void Messenger::publishSpiCanBusOwner(SpiCanBusOwner owner)
{
    spiCanBusOwner = owner;
}

SpiCanBusOwner Messenger::readSpiCanBusOwner() const
{
    return spiCanBusOwner;
}

void Messenger::setCanActive(bool active)
{
    canActiveFlag = active;
}

bool Messenger::canActive() const
{
    return canActiveFlag;
}

void Messenger::requestCalibration(CalType type)
{
    calType    = type;
    calPending = true;
}

bool Messenger::calibrationPending() const
{
    return calPending;
}

CalType Messenger::calibrationRequest() const
{
    return calType;
}

void Messenger::clearCalibrationRequest()
{
    calPending = false;
}

void Messenger::publishCalView(const CalView& view)
{
    calViewState = view;
}

const CalView& Messenger::calView() const
{
    return calViewState;
}

void Messenger::postCalCommand(CalCmd cmd)
{
    calCmd = cmd;
}

CalCmd Messenger::takeCalCommand()
{
    CalCmd cmd = calCmd;
    calCmd = CalCmd::None;
    return cmd;
}

void Messenger::publishCanState(const CanState& st)
{
    canStateData = st;
}

const CanState& Messenger::canState() const
{
    return canStateData;
}

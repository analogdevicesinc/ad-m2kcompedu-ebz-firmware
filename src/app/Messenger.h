/*
 * Messenger.h
 * Scope: Single source of truth for state shared between app modules.
 *        Writers update it, readers read it and diff against their own
 *        last-applied snapshot.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MESSENGER_H
#define MESSENGER_H

#include <Arduino.h>

enum class SpiCanBusOwner : uint8_t {
    Can,
    SignalGen,
};

enum class SerialUser : uint8_t {
    Iio,
    Can,
    Audio,
};

enum class CalType : uint8_t {
    Offset,
    SineAmp,
    SqwAmp
};

enum class CalCmd : uint8_t {
    None,
    AdjustUp,
    AdjustDown,
    Next,
    Abort
};

struct CalView {
    bool    active = false;
    bool    done   = false;
    CalType type   = CalType::Offset;
    uint8_t step   = 0;
    int16_t value  = 0;

    bool operator!=(const CalView& o) const
    {
        return active != o.active || done != o.done || type != o.type ||
               step != o.step || value != o.value;
    }
};

struct CanState {
    bool     active  = false;
    bool     open    = false;
    uint32_t bitrate = 0;
    uint16_t rxCount = 0;
    uint16_t txCount = 0;
    uint16_t txFail  = 0;

    bool operator!=(const CanState& o) const
    {
        return active != o.active || open != o.open || bitrate != o.bitrate ||
               rxCount != o.rxCount || txCount != o.txCount || txFail != o.txFail;
    }
};

constexpr uint32_t ADC_FULL_SCALE_MV = 5000;
constexpr uint32_t ADC_MAX_COUNT     = 16383;

struct AppState {
    uint8_t  waveform     = 0;
    uint16_t amplitude    = 1000;
    uint8_t  freqOrder    = 1;
    float    frequency    = 1.0f;
    int16_t  dcOffset     = 0;

    bool     pwmEnabled   = false;
    uint8_t  pwmFreqOrder = 0;
    float    pwmFrequency = 100.0f;
    uint8_t  dutyCycle    = 50;

    int16_t  calOffset    = 0;
    int16_t  calSine1V    = 0;
    int16_t  calSine10V   = 0;
    int16_t  calSqw1V     = 0;
    int16_t  calSqw10V    = 0;

    uint16_t voltmeterRaw[2] = {0};
    uint16_t voltmeterMv[2]  = {0};
    float voltmeterScale[2]  = {0};

    int16_t  audioRaw[2]     = {0};
    float    audioScale[2]   = {0};
};

class Messenger
{
    public:
        AppState&       state();
        const AppState& state() const;

        void           publishSpiCanBusOwner(SpiCanBusOwner owner);
        SpiCanBusOwner readSpiCanBusOwner() const;

        void setCanActive(bool active);
        bool canActive() const;

        void setAudioActive(bool active);
        bool audioActive() const;

        void setAudioStreaming(bool s);
        bool audioStreaming() const;

        SerialUser serialUser() const;
        bool       serialAvailable() const;

        void    requestCalibration(CalType type);
        bool    calibrationPending() const;
        CalType calibrationRequest() const;
        void    clearCalibrationRequest();

        void            publishCalView(const CalView& view);
        const CalView&  calView() const;

        void   postCalCommand(CalCmd cmd);
        CalCmd takeCalCommand();

        void            publishCanState(const CanState& st);
        const CanState& canState() const;

    private:
        AppState       appState;
        SpiCanBusOwner spiCanBusOwner = SpiCanBusOwner::Can;
        bool           canActiveFlag  = false;
        bool           audioActiveFlag = false;
        bool           audioStreamingFlag = false;
        SerialUser     serialUserState = SerialUser::Iio;

        bool    calPending = false;
        CalType calType    = CalType::Offset;
        CalView calViewState;
        CalCmd  calCmd     = CalCmd::None;
        CanState canStateData;
};

#endif

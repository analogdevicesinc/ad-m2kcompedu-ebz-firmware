/*
 * CanMessenger.h
 * Scope: slcan (LAWICEL ASCII) bridge between the USB serial port and the CAN
 *        bus. Active only while the bus owner is Can; releases the peripheral
 *        when ownership returns to the signal generator.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAN_MESSENGER_H
#define CAN_MESSENGER_H

#include <Arduino.h>
#include <Arduino_CAN.h>
#include "Messenger.h"

class CanMessenger
{
    public:
        CanMessenger(Messenger& messenger);

        void init();
        void run();

        bool     active() const      { return _active; }
        bool     channelOpen() const { return _open; }
        uint16_t rxCount() const     { return _rxCount; }
        uint16_t txCount() const     { return _txCount; }
        uint16_t txFailCount() const { return _txFail; }
        char     lastCmd() const     { return _lastCmd; }

    private:
        void start();
        void stop();
        void handleCommand();
        bool openChannel();
        void closeChannel();
        bool setBitrate(char sel);
        bool transmit(bool extended);
        void emitFrame(const CanMsg& msg);
        void publishView();

        void ok()  { Serial.write('\r'); }
        void err() { Serial.write('\a'); }

        Messenger& messenger;

        bool _active;
        bool _open;
        bool _timestamp;
        bool _canInit;
        uint32_t _bitrate;
        uint32_t _initBitrate;

        char _rxBuf[32];
        uint8_t _rxLen;

        // Diagnostics only (mirrored to the OLED, never to the serial stream).
        uint16_t _rxCount;          // CAN frames drained from the bus
        uint16_t _txCount;          // t/T frames written to the bus OK
        uint16_t _txFail;           // t/T frames that failed to send
        char     _lastCmd;          // last slcan command char handled
};

#endif

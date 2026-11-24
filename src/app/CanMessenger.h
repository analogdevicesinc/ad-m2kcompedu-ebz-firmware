/*
 * CanMessenger.h
 * Scope: slcan (LAWICEL ASCII) bridge between the USB serial port and the CAN
 *        bus. Non-blocking; pumped from the main tick. Only valid in CAN mode
 *        (shares pins 10/13 with the AD9833).
 *
 * Commands (CR-terminated): S4/S5/S6/S8 set bitrate, O/L open, C close,
 * t/T transmit std/ext frame, F flags, V version, N serial, Zn timestamp.
 * Replies: <CR> ok, <BEL> error; received frames are echoed as t/T lines.
 * Works with slcand, python-can (slcan) and SavvyCAN.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAN_MESSENGER_H
#define CAN_MESSENGER_H

#include <Arduino.h>
#include <Arduino_CAN.h>

class CanMessenger
{
    public:
        CanMessenger();

        // Enter/leave slcan mode. start() claims nothing until the host opens
        // the channel with 'O'; stop() releases the CAN peripheral.
        void start();
        void stop();
        bool active() const { return _active; }

        // Pump RX (CAN -> serial) and commands/TX (serial -> CAN). Call every tick.
        void tick();

        // Out-of-band diagnostics for the OLED SLCAN screen (never touch the
        // serial byte stream, so safe to read while slcan is live).
        bool channelOpen()      const { return _open; }
        uint16_t rxCount()      const { return _rxCount; }
        uint16_t txCount()      const { return _txCount; }
        uint16_t txFailCount()  const { return _txFail; }
        char lastCmd()          const { return _lastCmd; }

    private:
        void handleCommand();
        bool openChannel();
        void closeChannel();
        bool setBitrate(char sel);
        bool transmit(bool extended);
        void emitFrame(const CanMsg& msg);

        void ok()  { Serial.write('\r'); }
        void err() { Serial.write('\a'); }

        bool _active;
        bool _open;                 // logical slcan channel state (O/C)
        bool _timestamp;
        bool _canInit;              // CAN peripheral is begun; survives slcan close
        uint32_t _bitrate;          // bits/sec, applied on open
        uint32_t _initBitrate;      // bitrate the peripheral was begun at

        char _rxBuf[32];
        uint8_t _rxLen;

        // Diagnostics only.
        uint16_t _rxCount;          // CAN frames drained (whether forwarded or not)
        uint16_t _txCount;          // t/T frames written to the bus OK
        uint16_t _txFail;           // t/T frames that failed to send
        char _lastCmd;              // last slcan command char handled
};

#endif

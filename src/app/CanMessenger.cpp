/*
 * CanMessenger.cpp
 * Scope: slcan (LAWICEL) protocol implementation. See CanMessenger.h.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "CanMessenger.h"

// TEMPORARY bring-up diagnostics. Set to 0 (or delete the guarded blocks)
// before using with canopen/slcan for real -- these prints corrupt the
// slcan byte stream.
#define CANMSG_DEBUG 0

static int hexNibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parseHex(const char* p, uint8_t count, uint32_t& val)
{
    val = 0;
    for (uint8_t i = 0; i < count; i++)
    {
        int n = hexNibble(p[i]);
        if (n < 0) return false;
        val = (val << 4) | (uint32_t)n;
    }
    return true;
}

static CanBitRate toBitRate(uint32_t bps)
{
    switch (bps)
    {
        case 125000:  return CanBitRate::BR_125k;
        case 500000:  return CanBitRate::BR_500k;
        case 1000000: return CanBitRate::BR_1000k;
        default:      return CanBitRate::BR_250k;
    }
}

CanMessenger::CanMessenger()
    : _active(false), _open(false), _timestamp(false), _canInit(false),
      _bitrate(250000), _initBitrate(0), _rxLen(0),
      _rxCount(0), _txCount(0), _txFail(0), _lastCmd(0)
{
}

void CanMessenger::start()
{
    _active = true;
    _open = false;
    _timestamp = false;
    _rxLen = 0;
    _rxCount = 0;
    _txCount = 0;
    _txFail = 0;
    _lastCmd = 0;
#if CANMSG_DEBUG
    // Probe A: proves Serial TX works and start() actually ran.
    Serial.print("\r\n[CANMSG] started - type V then Enter\r\n");
#endif
}

void CanMessenger::stop()
{
    if (!_active) return;
    // Leaving slcan mode for good: fully release the CAN peripheral so pins
    // 10/13 return to the AD9833 signal generator.
    if (_canInit)
    {
        CAN.end();
        _canInit = false;
    }
    _open = false;
    _active = false;
}

void CanMessenger::tick()
{
    if (!_active) return;

    // Drain the RX FIFO whenever the peripheral is up, even if the slcan
    // channel is logically closed. Two reasons:
    //   1. It keeps the controller ACKing the bus between host sessions, so the
    //      drive never drifts to error-passive/bus-off for lack of an ACK peer.
    //   2. It stops the hardware RX FIFO overflowing during the (multi-second)
    //      gap between one python-can script sending 'C' and the next sending
    //      'O' -- an overflow there wedged reception on the next session.
    // Frames are only forwarded to the host while the channel is open; the rest
    // are discarded but still count toward the diagnostic rx tally.
    if (_canInit)
    {
        while (CAN.available())
        {
            CanMsg const msg = CAN.read();
            _rxCount++;
#if CANMSG_DEBUG
            // Probe D: a frame actually arrived from the CAN bus (i.e. the drive
            // answered). Also reports (bool)Serial so we can tell if the emit
            // guard would drop it.
            Serial.print("[CANMSG] CAN rx id=0x");
            Serial.print((unsigned long)msg.getStandardId(), HEX);
            Serial.print(" len=");
            Serial.print((unsigned)msg.data_length);
            Serial.print(" serOK=");
            Serial.print((bool)Serial ? '1' : '0');
            Serial.print("\r\n");
#endif
            if (_open)
            {
                emitFrame(msg);
            }
        }
    }

    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\n') continue;
        if (c == '\r')
        {
            _rxBuf[_rxLen] = '\0';
            handleCommand();
            _rxLen = 0;
        }
        else if (_rxLen < sizeof(_rxBuf) - 1)
        {
            _rxBuf[_rxLen++] = c;
        }
    }
}

void CanMessenger::handleCommand()
{
    if (_rxLen == 0) return;

    _lastCmd = _rxBuf[0];   // diagnostics: last slcan command char seen

#if CANMSG_DEBUG
    // Probe C: proves Serial RX + line framing work (a full \r-terminated
    // command reached here). If B prints but C never does when you type,
    // RX is the broken link (chars not arriving, or no CR terminator).
    Serial.print("[CANMSG] rx: '");
    Serial.print(_rxBuf);
    Serial.print("'\r\n");
#endif

    switch (_rxBuf[0])
    {
        case 'S':                       // Sn - set bitrate (channel closed)
            (_rxLen == 2 && setBitrate(_rxBuf[1])) ? ok() : err();
            break;

        case 'O':                       // open (normal)
        case 'L':                       // open (listen-only: best effort)
            openChannel() ? ok() : err();
            break;

        case 'C':                       // close
            closeChannel();
            ok();
            break;

        case 't':
            if (transmit(false)) { _txCount++; Serial.write('z'); ok(); } else { _txFail++; err(); }
            break;

        case 'T':
            if (transmit(true)) { _txCount++; Serial.write('Z'); ok(); } else { _txFail++; err(); }
            break;

        case 'F':                       // status flags: none set
            Serial.print("F00");
            ok();
            break;

        case 'Z':                       // Zn - timestamp on/off
            if (_rxLen == 2 && (_rxBuf[1] == '0' || _rxBuf[1] == '1'))
            {
                _timestamp = (_rxBuf[1] == '1');
                ok();
            }
            else err();
            break;

        case 'V': Serial.print("V1010"); ok(); break;   // version
        case 'N': Serial.print("NA123"); ok(); break;   // serial number

        case 'M':                       // acceptance code/mask - accept all
        case 'm':
            ok();
            break;

        default:
            err();
            break;
    }
}

bool CanMessenger::setBitrate(char sel)
{
    switch (sel)
    {
        case '4': _bitrate = 125000;  return true;
        case '5': _bitrate = 250000;  return true;
        case '6': _bitrate = 500000;  return true;
        case '8': _bitrate = 1000000; return true;
        default:  return false;         // S0-3/S7 unsupported by Arduino_CAN
    }
}

bool CanMessenger::openChannel()
{
    if (_open) return true;

    // Only (re)begin the peripheral if it isn't already up, or the bitrate
    // changed. Keeping it initialised across slcan close/open cycles avoids
    // tearing down and rebuilding the CAN controller on every host session
    // (one canopen script per run), and keeps us ACKing the bus in between so
    // the drive never drifts to error-passive/bus-off for lack of an ACK peer.
    if (_canInit && _initBitrate != _bitrate)
    {
        CAN.end();
        _canInit = false;
    }

    if (!_canInit)
    {
        if (!CAN.begin(toBitRate(_bitrate)))
        {
#if CANMSG_DEBUG
            Serial.print("[CANMSG] CAN.begin FAILED\r\n");
#endif
            return false;
        }
        _canInit = true;
        _initBitrate = _bitrate;
    }

    // Discard frames buffered while the channel was logically closed so the
    // new session starts clean.
    while (CAN.available())
    {
        (void)CAN.read();
    }

    _open = true;
#if CANMSG_DEBUG
    Serial.print("[CANMSG] CAN opened @");
    Serial.print((unsigned long)_bitrate);
    Serial.print("\r\n");
#endif
    return true;
}

void CanMessenger::closeChannel()
{
    // Logical close only. The peripheral stays begun (see openChannel); full
    // teardown happens in stop() when we leave slcan mode entirely.
    _open = false;
}

bool CanMessenger::transmit(bool extended)
{
    if (!_open) return false;

    const uint8_t idLen = extended ? 8 : 3;
    if (_rxLen < 1 + idLen + 1) return false;

    uint32_t id;
    if (!parseHex(&_rxBuf[1], idLen, id)) return false;

    char dlcChar = _rxBuf[1 + idLen];
    if (dlcChar < '0' || dlcChar > '8') return false;
    uint8_t dlc = (uint8_t)(dlcChar - '0');

    if (_rxLen != 1 + idLen + 1 + dlc * 2) return false;

    uint8_t data[8];
    const char* d = &_rxBuf[1 + idLen + 1];
    for (uint8_t i = 0; i < dlc; i++)
    {
        int hi = hexNibble(d[i * 2]);
        int lo = hexNibble(d[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        data[i] = (uint8_t)((hi << 4) | lo);
    }

    CanMsg const msg(extended ? CanExtendedId(id) : CanStandardId(id), dlc, data);
    return CAN.write(msg) >= 0;
}

void CanMessenger::emitFrame(const CanMsg& msg)
{
    char buf[40];
    uint8_t pos = 0;

    if (msg.isExtendedId())
        pos += snprintf(&buf[pos], sizeof(buf) - pos, "T%08lX%u",
                        (unsigned long)msg.getExtendedId(),
                        (unsigned)msg.data_length);
    else
        pos += snprintf(&buf[pos], sizeof(buf) - pos, "t%03lX%u",
                        (unsigned long)msg.getStandardId(),
                        (unsigned)msg.data_length);

    for (uint8_t i = 0; i < msg.data_length && pos < sizeof(buf) - 3; i++)
        pos += snprintf(&buf[pos], sizeof(buf) - pos, "%02X", msg.data[i]);

    if (_timestamp && pos < sizeof(buf) - 5)
        pos += snprintf(&buf[pos], sizeof(buf) - pos, "%04X",
                        (unsigned)(millis() & 0xFFFF));

    Serial.write((const uint8_t*)buf, pos);
    Serial.write('\r');
}

/*
 * CanMessenger.cpp
 * Scope: slcan (LAWICEL) protocol implementation. See CanMessenger.h.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "CanMessenger.h"

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

CanMessenger::CanMessenger(Messenger& messenger)
    : messenger(messenger),
      _active(false), _open(false), _timestamp(false), _canInit(false),
      _bitrate(250000), _initBitrate(0), _rxLen(0),
      _rxCount(0), _txCount(0), _txFail(0), _lastCmd(0)
{
}

void CanMessenger::init()
{
}

void CanMessenger::run()
{
    bool canOwns = (messenger.readSpiCanBusOwner() == SpiCanBusOwner::Can);

    if (canOwns && !_active)
    {
        start();
    }
    else if (!canOwns && _active)
    {
        stop();
    }

    if (_active)
    {
        if (_canInit)
        {
            while (CAN.available())
            {
                CanMsg const msg = CAN.read();
                _rxCount++;
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

    publishView();
}

void CanMessenger::publishView()
{
    CanState v;
    v.active  = _active;
    v.open    = _open;
    v.bitrate = _bitrate;
    v.rxCount = _rxCount;
    v.txCount = _txCount;
    v.txFail  = _txFail;
    messenger.publishCanState(v);
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
}

void CanMessenger::stop()
{
    if (!_active) return;
    if (_canInit)
    {
        CAN.end();
        _canInit = false;
    }
    _open = false;
    _active = false;
}

void CanMessenger::handleCommand()
{
    if (_rxLen == 0) return;

    _lastCmd = _rxBuf[0];

    switch (_rxBuf[0])
    {
        case 'S':
            if (_rxLen == 2 && setBitrate(_rxBuf[1]))
            {
                ok();
            }
            else
            {
                err();
            }
            break;

        case 'O':
        case 'L':
            if (openChannel())
            {
                ok();
            }
            else
            {
                err();
            }
            break;

        case 'C':
            closeChannel();
            ok();
            break;

        case 't':
            if (transmit(false)) { _txCount++; Serial.write('z'); ok(); } else { _txFail++; err(); }
            break;

        case 'T':
            if (transmit(true)) { _txCount++; Serial.write('Z'); ok(); } else { _txFail++; err(); }
            break;

        case 'F':
            Serial.print("F00");
            ok();
            break;

        case 'Z':
            if (_rxLen == 2 && (_rxBuf[1] == '0' || _rxBuf[1] == '1'))
            {
                _timestamp = (_rxBuf[1] == '1');
                ok();
            }
            else err();
            break;

        case 'V': Serial.print("V1010"); ok(); break;
        case 'N': Serial.print("NA123"); ok(); break;

        case 'M':
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
        default:  return false;
    }
}

bool CanMessenger::openChannel()
{
    if (_open) return true;

    if (_canInit && _initBitrate != _bitrate)
    {
        CAN.end();
        _canInit = false;
    }

    if (!_canInit)
    {
        if (!CAN.begin(toBitRate(_bitrate)))
        {
            return false;
        }
        _canInit = true;
        _initBitrate = _bitrate;
    }

    while (CAN.available())
    {
        (void)CAN.read();
    }

    _open = true;
    return true;
}

void CanMessenger::closeChannel()
{
    _open = false;
}

bool CanMessenger::transmit(bool extended)
{
    if (!_open) return false;

    uint8_t idLen = 3;
    if (extended)
    {
        idLen = 8;
    }
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

    uint32_t canId = CanStandardId(id);
    if (extended)
    {
        canId = CanExtendedId(id);
    }
    CanMsg const msg(canId, dlc, data);
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

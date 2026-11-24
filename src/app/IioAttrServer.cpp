/*
 * IioAttrServer.cpp
 * Scope: Serves app Params as IIOD channel attributes; owns the underlying
 * IioServer transport.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "IioAttrServer.h"
#include "Controler.h"
#include "../iio/iiod.h"
#include "../iio/iio_server.h"

#include "context_xml.h"

/*
 * Mappings between channel, attribute and parameter id
 * Frequency attributes are handled separately to handle mantissa + exponent
 */
struct ParamAttrMap
{
    const char* channel;
    const char* attr;
    uint8_t idx;
};

static const ParamAttrMap paramAttrMap[] = {
    {"altvoltage0", "wavetype", P_WAVE},
    {"altvoltage0", "amplitude", P_AMPL},
    {"altvoltage0", "offset", P_DC},
    {"altvoltage1", "enable", P_PWM_EN},
    {"altvoltage1", "duty_cycle", P_PWM_DUTY},
};

static int find_param_index(const char* channel, const char* attr)
{
    for (uint32_t i = 0; i < sizeof(paramAttrMap) / sizeof(paramAttrMap[0]); i++)
    {
        if (strcmp(paramAttrMap[i].channel, channel) == 0 &&
            strcmp(paramAttrMap[i].attr, attr) == 0)
            return paramAttrMap[i].idx;
    }

    return -1;
}

/* IIOD sends and receives everything as strings; handling funcs are necessary */
static int param_to_string(const Param* p, char* buf, uint32_t len)
{
    if (p->isFloat)
        return snprintf(buf, len, "%.3f", *(float*)p->value);
    if (p->min >= 0 && p->max <= 255)
        return snprintf(buf, len, "%u", (unsigned)*(uint8_t*)p->value);
    if (p->min >= 0 && p->max <= 65535)
        return snprintf(buf, len, "%u", (unsigned)*(uint16_t*)p->value);

    return snprintf(buf, len, "%d", (int)*(int16_t*)p->value);
}

static void string_to_param(Param* p, const char* buf)
{
    if (p->isFloat)
    {
        *(float*)p->value = atof(buf);
        return;
    }

    long val = atol(buf);
    if (val < p->min)
        val = p->min;
    if (val > p->max)
        val = p->max;

    if (p->min >= 0 && p->max <= 255)
        *(uint8_t*)p->value = (uint8_t)val;
    else if (p->min >= 0 && p->max <= 65535)
        *(uint16_t*)p->value = (uint16_t)val;
    else
        *(int16_t*)p->value = (int16_t)val;
}


IioAttrServer::IioAttrServer(Param* params, bool* refreshFlags)
    : params(params), refreshFlags(refreshFlags)
{
}

float IioAttrServer::getFrequencyHz(uint8_t valIdx, uint8_t ordIdx)
{
    float freq = *(float*)params[valIdx].value;
    uint8_t order = *(uint8_t*)params[ordIdx].value;

    if (order == 1)
        freq *= 1000.0f;
    else if (order == 2)
        freq *= 1000000.0f;

    return freq;
}

void IioAttrServer::setFrequencyHz(uint8_t valIdx, uint8_t ordIdx, float minHz, float hz)
{
    uint8_t order = 0;

    if (hz < minHz)
        hz = minHz;

    while (hz >= 1000.0f && order < 2)
    {
        hz /= 1000.0f;
        order++;
    }
    if (order == 2 && hz > 2.0f)
        hz = 2.0f;

    *(float*)params[valIdx].value = hz;
    *(uint8_t*)params[ordIdx].value = order;

    refreshFlags[valIdx] = true;
    refreshFlags[ordIdx] = true;
}

int IioAttrServer::readAttr(const char* device, iiod_attr* attr, char* buf, uint32_t len)
{
    int idx;

    if (strcmp(device, IIO_DEVICE_NAME) != 0)
        return -ENODEV;
    if (attr->type != IIO_ATTR_TYPE_CH_OUT)
        return -EINVAL;
    if (!params)
        return -EINVAL;

    if (strcmp(attr->name, "frequency") == 0)
    {
        if (strcmp(attr->channel, "altvoltage0") == 0)
            return snprintf(buf, len, "%.3f", getFrequencyHz(P_FREQ_VAL, P_FREQ_ORD));
        if (strcmp(attr->channel, "altvoltage1") == 0)
            return snprintf(buf, len, "%.3f", getFrequencyHz(P_PWM_FREQ_VAL, P_PWM_FREQ_ORD));
        return -EINVAL;
    }

    idx = find_param_index(attr->channel, attr->name);
    if (idx < 0)
        return -EINVAL;

    return param_to_string(&params[idx], buf, len);
}

int IioAttrServer::writeAttr(const char* device, iiod_attr* attr, char* buf, uint32_t len)
{
    int idx;

    if (strcmp(device, IIO_DEVICE_NAME) != 0)
        return -ENODEV;
    if (attr->type != IIO_ATTR_TYPE_CH_OUT)
        return -EINVAL;
    if (!params || !refreshFlags)
        return -EINVAL;

    if (strcmp(attr->name, "frequency") == 0)
    {
        if (strcmp(attr->channel, "altvoltage0") == 0)
        {
            setFrequencyHz(P_FREQ_VAL, P_FREQ_ORD, 10.0f, atof(buf));
            return (int)len;
        }
        if (strcmp(attr->channel, "altvoltage1") == 0)
        {
            setFrequencyHz(P_PWM_FREQ_VAL, P_PWM_FREQ_ORD, 1.0f, atof(buf));
            return (int)len;
        }
        return -EINVAL;
    }

    idx = find_param_index(attr->channel, attr->name);
    if (idx < 0)
        return -EINVAL;

    string_to_param(&params[idx], buf);
    refreshFlags[idx] = true;

    return (int)len;
}

const char* IioAttrServer::xml()
{
    return context_xml;
}

uint32_t IioAttrServer::xmlLen()
{
    return sizeof(context_xml) - 1;
}

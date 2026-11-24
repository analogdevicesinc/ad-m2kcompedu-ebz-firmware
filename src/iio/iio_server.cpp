/*
 * iio_server.cpp
 * Scope: Implements the IIOD server platform layer — UART transport, and
 * attribute read/write bridging between IIOD channels and app Params.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "iiod.h"
#include "iio_server.h"
#include "../app/Controler.h"

/*
 * Platform HAL for the IIOD server
 * Separated out of iiod_send and iiod_recv for clarity
 */
static int platform_uart_tx(const uint8_t *buf, uint32_t len)
{
	const uint32_t chunk = len < 32 ? len : 32;

	return (int)Serial.write(buf, chunk);
}

static int platform_uart_rx(uint8_t *buf, uint32_t len)
{
	int available = Serial.available();

	if (available <= 0)
		return 0;

	if ((uint32_t)available < len)
		len = (uint32_t)available;

	return (int)Serial.readBytes((char *)buf, len);
}

/*
 * IIOD server communication functions
 */
static int iiod_send(struct iiod_ctx *ctx, uint8_t *buf, uint32_t len)
{
	return platform_uart_tx(buf, len);
}

static int iiod_recv(struct iiod_ctx *ctx, uint8_t *buf, uint32_t len)
{
	return platform_uart_rx(buf, len);
}

static Param *app_params;
static bool *app_refresh_flags;

/*
 * Mappings between channel, attribute and parameter id
 * Frequency attributes are handled separately to handle mantissa + exponent
 */
struct ParamAttrMap
{
	const char *channel;
	const char *attr;
	uint8_t idx;
};

static const ParamAttrMap paramAttrMap[] = {
	{"altvoltage0", "wavetype", P_WAVE},
	{"altvoltage0", "amplitude", P_AMPL},
	{"altvoltage0", "offset", P_DC},
	{"altvoltage1", "enable", P_PWM_EN},
	{"altvoltage1", "duty_cycle", P_PWM_DUTY},
};

static int find_param_index(const char *channel, const char *attr)
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
static int param_to_string(const Param *p, char *buf, uint32_t len)
{
	if (p->isFloat)
		return snprintf(buf, len, "%.3f", *(float *)p->value);
	if (p->min >= 0 && p->max <= 255)
		return snprintf(buf, len, "%u", (unsigned)*(uint8_t *)p->value);
	if (p->min >= 0 && p->max <= 65535)
		return snprintf(buf, len, "%u", (unsigned)*(uint16_t *)p->value);

	return snprintf(buf, len, "%d", (int)*(int16_t *)p->value);
}

static void string_to_param(Param *p, const char *buf)
{
	if (p->isFloat)
	{
		*(float *)p->value = atof(buf);
		return;
	}

	long val = atol(buf);
	if (val < p->min)
		val = p->min;
	if (val > p->max)
		val = p->max;

	if (p->min >= 0 && p->max <= 255)
		*(uint8_t *)p->value = (uint8_t)val;
	else if (p->min >= 0 && p->max <= 65535)
		*(uint16_t *)p->value = (uint16_t)val;
	else
		*(int16_t *)p->value = (int16_t)val;
}

static float get_frequency_hz(uint8_t valIdx, uint8_t ordIdx)
{
	float freq = *(float *)app_params[valIdx].value;
	uint8_t order = *(uint8_t *)app_params[ordIdx].value;

	if (order == 1)
		freq *= 1000.0f;
	else if (order == 2)
		freq *= 1000000.0f;

	return freq;
}

static void set_frequency_hz(uint8_t valIdx, uint8_t ordIdx, float minHz, float hz)
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

	*(float *)app_params[valIdx].value = hz;
	*(uint8_t *)app_params[ordIdx].value = order;

	app_refresh_flags[valIdx] = true;
	app_refresh_flags[ordIdx] = true;
}

static int iiod_read_attr(struct iiod_ctx *ctx, const char *device,
						  struct iiod_attr *attr, char *buf, uint32_t len)
{
	int idx;

	if (strcmp(device, IIO_DEVICE_NAME) != 0)
		return -ENODEV;
	if (attr->type != IIO_ATTR_TYPE_CH_OUT)
		return -EINVAL;
	if (!app_params)
		return -EINVAL;

	if (strcmp(attr->name, "frequency") == 0)
	{
		if (strcmp(attr->channel, "altvoltage0") == 0)
			return snprintf(buf, len, "%.3f", get_frequency_hz(P_FREQ_VAL, P_FREQ_ORD));
		if (strcmp(attr->channel, "altvoltage1") == 0)
			return snprintf(buf, len, "%.3f", get_frequency_hz(P_PWM_FREQ_VAL, P_PWM_FREQ_ORD));
		return -EINVAL;
	}

	idx = find_param_index(attr->channel, attr->name);
	if (idx < 0)
		return -EINVAL;

	return param_to_string(&app_params[idx], buf, len);
}

static int iiod_write_attr(struct iiod_ctx *ctx, const char *device,
						   struct iiod_attr *attr, char *buf, uint32_t len)
{
	int idx;

	if (strcmp(device, IIO_DEVICE_NAME) != 0)
		return -ENODEV;
	if (attr->type != IIO_ATTR_TYPE_CH_OUT)
		return -EINVAL;
	if (!app_params || !app_refresh_flags)
		return -EINVAL;

	if (strcmp(attr->name, "frequency") == 0)
	{
		if (strcmp(attr->channel, "altvoltage0") == 0)
		{
			set_frequency_hz(P_FREQ_VAL, P_FREQ_ORD, 10.0f, atof(buf));
			return (int)len;
		}
		if (strcmp(attr->channel, "altvoltage1") == 0)
		{
			set_frequency_hz(P_PWM_FREQ_VAL, P_PWM_FREQ_ORD, 1.0f, atof(buf));
			return (int)len;
		}
		return -EINVAL;
	}

	idx = find_param_index(attr->channel, attr->name);
	if (idx < 0)
		return -EINVAL;

	string_to_param(&app_params[idx], buf);
	app_refresh_flags[idx] = true;

	return (int)len;
}

static const char context_xml[] =
	"<?xml version=\"1.0\" encoding=\"utf-8\"?>"
	"<!DOCTYPE context ["
	"<!ELEMENT context (device)*>"
	"<!ATTLIST context name CDATA #REQUIRED>"
	"<!ATTLIST context description CDATA #IMPLIED>"
	"<!ELEMENT device (channel)*>"
	"<!ATTLIST device id CDATA #REQUIRED>"
	"<!ATTLIST device name CDATA #IMPLIED>"
	"<!ELEMENT channel (scan-element?, attribute*)>"
	"<!ATTLIST channel id CDATA #REQUIRED>"
	"<!ATTLIST channel type (input|output) #REQUIRED>"
	"<!ATTLIST channel name CDATA #IMPLIED>"
	"<!ELEMENT scan-element EMPTY>"
	"<!ATTLIST scan-element index CDATA #REQUIRED>"
	"<!ATTLIST scan-element format CDATA #REQUIRED>"
	"<!ELEMENT attribute EMPTY>"
	"<!ATTLIST attribute name CDATA #REQUIRED>"
	"<!ATTLIST attribute filename CDATA #IMPLIED>"
	"]>"
	"<context name=\"arduino-uart\" description=\"m2k-compedu signal generator\">"
	"<device id=\"" IIO_DEVICE_NAME "\" name=\"" IIO_DEVICE_NAME "\">"
	"<channel id=\"altvoltage0\" type=\"output\">"
	"<attribute name=\"wavetype\" filename=\"out_altvoltage0_wavetype\"/>"
	"<attribute name=\"amplitude\" filename=\"out_altvoltage0_amplitude\"/>"
	"<attribute name=\"offset\" filename=\"out_altvoltage0_offset\"/>"
	"<attribute name=\"frequency\" filename=\"out_altvoltage0_frequency\"/>"
	"</channel>"
	"<channel id=\"altvoltage1\" type=\"output\">"
	"<attribute name=\"enable\" filename=\"out_altvoltage1_enable\"/>"
	"<attribute name=\"duty_cycle\" filename=\"out_altvoltage1_duty_cycle\"/>"
	"<attribute name=\"frequency\" filename=\"out_altvoltage1_frequency\"/>"
	"</channel>"
	"</device>"
	"</context>";

static struct iiod_desc *desc;
static uint32_t conn_id;
static char payload_buf[256];
static bool iio_disabled;

void iio_server_begin(Param *params, bool *refreshFlags)
{
	struct iiod_ops ops = {};
	struct iiod_init_param param = {};
	struct iiod_conn_data conn_data;
	int32_t ret;

	app_params = params;
	app_refresh_flags = refreshFlags;

	ops.send = iiod_send;
	ops.recv = iiod_recv;
	ops.read_attr = iiod_read_attr;
	ops.write_attr = iiod_write_attr;

	param.ops = &ops;
	param.instance = NULL;
	param.xml = (char *)context_xml;
	param.xml_len = sizeof(context_xml) - 1;
	param.phy_type = USE_UART;

	ret = iiod_init(&desc, &param);
	if (ret)
	{
		iio_disabled = true;
		return;
	}

	conn_data.conn = NULL;		 /* opaque handle echoed back in iiod_ctx.conn */
	conn_data.buf = payload_buf; /* must fit the largest attribute value/line */
	conn_data.len = sizeof(payload_buf);

	iiod_conn_add(desc, &conn_data, &conn_id);
}

void iio_server_poll()
{
	if (!iio_disabled)
	{
		int32_t ret;

		ret = iiod_conn_step(desc, conn_id);
		if (ret && ret != -EAGAIN)
		{
			/*
			 * Client sent EXIT, or a real I/O error occurred. Drop and
			 * re-accept the connection so a new client can connect.
			 */
			struct iiod_conn_data conn_data;

			iiod_conn_remove(desc, conn_id, &conn_data);
			iiod_conn_add(desc, &conn_data, &conn_id);
		}
	}
}

void iio_server_enable()
{
	iio_disabled = false;
}

void iio_server_disable()
{
	iio_disabled = true;
}

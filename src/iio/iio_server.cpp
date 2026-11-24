/*
 * iio_server.cpp
 * Scope: Implements the IIOD server platform layer — UART transport and
 * connection lifecycle. Attribute semantics are supplied by the caller.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <Arduino.h>
#include <errno.h>

#include "iiod_cxx.h"
#include "iio_server.h"

/*
 * Platform HAL for the IIOD server
 * Separated out of the send/recv callbacks for clarity
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

int IioServer::send(uint8_t *buf, uint32_t len)
{
	return platform_uart_tx(buf, len);
}

int IioServer::recv(uint8_t *buf, uint32_t len)
{
	return platform_uart_rx(buf, len);
}

int IioServer::sendTrampoline(iiod_ctx *ctx, uint8_t *buf, uint32_t len)
{
	return static_cast<IioServer *>(ctx->instance)->send(buf, len);
}

int IioServer::recvTrampoline(iiod_ctx *ctx, uint8_t *buf, uint32_t len)
{
	return static_cast<IioServer *>(ctx->instance)->recv(buf, len);
}

int IioServer::readAttrTrampoline(iiod_ctx *ctx, const char *device, iiod_attr *attr,
				  char *buf, uint32_t len)
{
	return static_cast<IioServer *>(ctx->instance)->readAttr(device, attr, buf, len);
}

int IioServer::writeAttrTrampoline(iiod_ctx *ctx, const char *device, iiod_attr *attr,
				   char *buf, uint32_t len)
{
	return static_cast<IioServer *>(ctx->instance)->writeAttr(device, attr, buf, len);
}

void IioServer::begin(const char *xml, uint32_t xmlLen)
{
	struct iiod_ops ops = {};
	struct iiod_init_param param = {};
	struct iiod_conn_data conn_data;
	int32_t ret;

	ops.send = sendTrampoline;
	ops.recv = recvTrampoline;
	ops.read_attr = readAttrTrampoline;
	ops.write_attr = writeAttrTrampoline;

	param.ops = &ops;
	param.instance = this;
	param.xml = (char *)xml;
	param.xml_len = xmlLen;
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

void IioServer::poll()
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

void IioServer::enable()
{
	iio_disabled = false;
}

void IioServer::disable()
{
	iio_disabled = true;
}

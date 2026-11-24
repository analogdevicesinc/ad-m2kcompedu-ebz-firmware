/*
 * iio_server.h - Arduino-side iiod server wrapper
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IIO_SERVER_H
#define IIO_SERVER_H

#include <stdint.h>

struct iiod_desc;
struct iiod_ctx;
struct iiod_attr;

class IioServer
{
	public:
		void begin(const char *xml, uint32_t xmlLen);
		void poll();
		void enable();
		void disable();
		virtual ~IioServer() = default;

	protected:
		virtual int readAttr(const char *device, iiod_attr *attr,
				     char *buf, uint32_t len) = 0;
		virtual int writeAttr(const char *device, iiod_attr *attr,
				      char *buf, uint32_t len) = 0;

		/* Default transport is UART; override to use a different one. */
		virtual int send(uint8_t *buf, uint32_t len);
		virtual int recv(uint8_t *buf, uint32_t len);

	private:
		/* Recover the instance from ctx->instance and forward the call. */
		static int readAttrTrampoline(iiod_ctx *ctx, const char *device,
					      iiod_attr *attr, char *buf, uint32_t len);
		static int writeAttrTrampoline(iiod_ctx *ctx, const char *device,
					       iiod_attr *attr, char *buf, uint32_t len);
		static int sendTrampoline(iiod_ctx *ctx, uint8_t *buf, uint32_t len);
		static int recvTrampoline(iiod_ctx *ctx, uint8_t *buf, uint32_t len);

		iiod_desc *desc = nullptr;
		uint32_t conn_id = 0;
		char payload_buf[256] = {};
		bool iio_disabled = false;
};

#endif

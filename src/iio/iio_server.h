/*
 * iio_server.h - Arduino-side iiod server wrapper
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IIO_SERVER_H
#define IIO_SERVER_H

#define IIO_DEVICE_NAME "m2kcompedu-ebz"

struct Param;

void iio_server_begin(Param* params, bool* refreshFlags);
void iio_server_poll(void);
void iio_server_enable(void);
void iio_server_disable(void);

#endif

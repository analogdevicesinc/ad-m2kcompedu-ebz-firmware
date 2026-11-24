/*
 * iio.h - Minimal shim satisfying no-OS iiod.c/iiod.h/iiod_private.h's
 * #include "iio.h", without pulling in the real no-OS iio.h (which drags in
 * iio_types.h, no_os_uart.h and tcp_socket.h). Only enum physical_link_type
 * is actually referenced by those files.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IIO_H
#define IIO_H

enum physical_link_type {
	USE_UART,
	USE_LOCAL_BACKEND,
	USE_NETWORK
};

#endif

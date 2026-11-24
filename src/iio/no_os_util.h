/*
 * no_os_util.h - Minimal shim satisfying no-OS iiod.c's
 * #include "no_os_util.h". Only no_os_min and NO_OS_ARRAY_SIZE are actually
 * referenced.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NO_OS_UTIL_H
#define NO_OS_UTIL_H

#define no_os_min(x, y)		(((x) < (y)) ? (x) : (y))
#define NO_OS_ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

#endif

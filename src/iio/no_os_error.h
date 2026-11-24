/*
 * no_os_error.h - Minimal shim satisfying no-OS iiod.c's
 * #include "no_os_error.h". Only NO_OS_IS_ERR_VALUE is actually referenced
 * directly, but iiod.c also relies on this header to pull in <errno.h> for
 * EINVAL/EAGAIN/etc. (the real no_os_error.h does the same).
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NO_OS_ERROR_H
#define NO_OS_ERROR_H

#include <errno.h>

#define NO_OS_IS_ERR_VALUE(x)	((x) < 0)

#endif

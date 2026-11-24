/*
 * iiod_cxx.h - C++ linkage wrapper for no-OS's iiod.h.
 *
 * Pristine no-OS iiod.h has no extern "C" guard (every in-tree no-OS
 * consumer is plain C). iiod.c is compiled as C, so any C++ translation
 * unit must include iiod.h under extern "C" to match that linkage. Include
 * this wrapper instead of "iiod.h" directly from .cpp files, so the
 * vendored header never needs to be touched.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IIOD_CXX_H
#define IIOD_CXX_H

extern "C" {
#include "iiod.h"
}

#endif

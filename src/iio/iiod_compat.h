/*
 * iiod_compat.h
 *
 * Standalone replacement for macros defined in no_os_error.h and
 * no_os_util.h, needed for iiod.c
 * 
 */

#ifndef IIOD_COMPAT_H
#define IIOD_COMPAT_H

#include <errno.h>
#include <stddef.h>

/* iiod.c checks return codes with this instead of comparing to 0 directly. */
#define NO_OS_IS_ERR_VALUE(x)	((x) < 0)

/* Only used to clamp read/write chunk sizes against the connection buffer. */
#define no_os_min(x, y)		((x) < (y) ? (x) : (y))

/* Only used for a compile-time array size check. */
#define NO_OS_ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

#endif /* IIOD_COMPAT_H */

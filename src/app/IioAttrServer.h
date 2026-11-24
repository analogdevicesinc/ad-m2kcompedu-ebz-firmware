/*
 * IioAttrServer.h
 * Scope: Serves app Params as IIOD channel attributes; owns the underlying
 * IioServer transport.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IIO_ATTR_SERVER_H
#define IIO_ATTR_SERVER_H

#include <stdint.h>

#include "../iio/iio_server.h"

struct Param;
struct iiod_attr;

#define IIO_DEVICE_NAME "m2kcompedu-ebz"

class IioAttrServer : public IioServer
{
    public:
        IioAttrServer(Param* params, bool* refreshFlags);

        static const char* xml();
        static uint32_t xmlLen();

    protected:
        int readAttr(const char* device, iiod_attr* attr, char* buf, uint32_t len) override;
        int writeAttr(const char* device, iiod_attr* attr, char* buf, uint32_t len) override;

    private:
        float getFrequencyHz(uint8_t valIdx, uint8_t ordIdx);
        void setFrequencyHz(uint8_t valIdx, uint8_t ordIdx, float minHz, float hz);

        Param* params;
        bool* refreshFlags;
};

#endif

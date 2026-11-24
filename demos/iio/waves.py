#!/usr/bin/env python3
"""Example: talk to an m2kcompedu-ebz device using libiio.
    Cycle through all available wave types and vary all
    modifiable parameters.

Serial connection details:
    port:      /dev/ttyACM0
    baudrate:  921600
    settings:  8N1

iio_info layout being targeted:
    m2kcompedu-ebz
        altvoltage0 (output): wavetype, amplitude, offset, frequency
        altvoltage1 (output): enable, duty_cycle, frequency
"""

import iio
import os
import sys
import time

# URI resolution order: CLI arg > IIO_URI env var > Linux default.
# On Windows use e.g. "serial:COM4,921600,8n1".
URI = (
    sys.argv[1]
    if len(sys.argv) > 1
    else os.environ.get("IIO_URI", "serial:/dev/ttyACM0,921600,8n1")
)
DEVICE_NAME = "m2kcompedu-ebz"


def set_attr(channel, name, value):
    channel.attrs[name].value = str(value)


def get_attr(channel, name):
    return channel.attrs[name].value


if __name__ == "__main__":
    ctx = iio.Context(URI)
    dev = ctx.find_device(DEVICE_NAME)
    if dev is None:
        raise Exception(f"Device {DEVICE_NAME} not found in context {URI}")

    altvoltage0 = dev.find_channel("altvoltage0", True)  # True = output
    altvoltage1 = dev.find_channel("altvoltage1", True)

    # Initially configure the analog output (altvoltage0) as a 1 kHz sine wave, 1V peak to peak with no DC offset
    set_attr(altvoltage0, "wavetype", 2)
    set_attr(altvoltage0, "amplitude", 1000)
    set_attr(altvoltage0, "frequency", 1000)
    set_attr(altvoltage0, "offset", 0)

    # Disable PWM output
    # set_attr(altvoltage1, "enable", 0)

    # Configure the PWM digital output (altvoltage1) as a 1 kH
    set_attr(altvoltage1, "duty_cycle", 0)
    set_attr(altvoltage1, "frequency", 1000.0)
    set_attr(altvoltage1, "enable", 1)

    try:
        while True:
            
            # For each wavetype:
            for wavetype in range (2, 5, 1):
                set_attr(altvoltage0, "wavetype", wavetype)

                # Fade LED in...
                for duty in range (0, 101, 5):
                    set_attr(altvoltage1, "duty_cycle", duty)
                    time.sleep(0.01)

                #... and out
                for duty in range (100, -1, -5):
                    set_attr(altvoltage1, "duty_cycle", duty)
                    time.sleep(0.01)

                # Vary frequency (stretch/compress horizontally)
                for freq in range (1000, 2550, 50):
                    set_attr(altvoltage0, "frequency", freq)
                    time.sleep(0.02)

                for freq in range (2500, 950, -50):
                    set_attr(altvoltage0, "frequency", freq)
                    time.sleep(0.02)

                # Vary amplitude (stretch/compress horizontally)
                for ampl in range (1000, 2550, 50):
                    set_attr(altvoltage0, "amplitude", ampl)
                    time.sleep(0.02)

                for ampl in range (2500, 950, -50):
                    set_attr(altvoltage0, "amplitude", ampl)
                    time.sleep(0.02)

                # Vary offset
                for off in range (0, 300, 50):
                    set_attr(altvoltage0, "offset", off)
                    time.sleep(0.02)

                for off in range (250, -300, -50):
                    set_attr(altvoltage0, "offset", off)
                    time.sleep(0.02)

                for off in range (-250, 50, 50):
                    set_attr(altvoltage0, "offset", off)
                    time.sleep(0.02)

                time.sleep(0.5)

    except KeyboardInterrupt:
        del ctx

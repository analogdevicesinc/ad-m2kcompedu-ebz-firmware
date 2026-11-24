#
# dsp.py
#
# Copyright (c) 2026 Analog Devices, Inc.
#
# SPDX-License-Identifier: Apache-2.0
#

import cmath
import math

FILTER_TYPES = [
    "Off", "Low-pass", "High-pass", "Band-pass", "Band-stop",
    "Peak", "Low Shelf", "High Shelf", "One-pole",
]


def biquad_coeffs(filter_type, fc, q, sample_rate, gain_db=0.0):
    w0 = 2.0 * math.pi * fc / sample_rate
    cosw0 = math.cos(w0)
    sinw0 = math.sin(w0)
    alpha = sinw0 / (2.0 * q)

    if filter_type == "Low-pass":
        b0 = (1.0 - cosw0) / 2.0
        b1 = 1.0 - cosw0
        b2 = b0
        a0 = 1.0 + alpha
        a1 = -2.0 * cosw0
        a2 = 1.0 - alpha
    elif filter_type == "High-pass":
        b0 = (1.0 + cosw0) / 2.0
        b1 = -(1.0 + cosw0)
        b2 = b0
        a0 = 1.0 + alpha
        a1 = -2.0 * cosw0
        a2 = 1.0 - alpha
    elif filter_type == "Band-pass":
        b0 = alpha
        b1 = 0.0
        b2 = -alpha
        a0 = 1.0 + alpha
        a1 = -2.0 * cosw0
        a2 = 1.0 - alpha
    elif filter_type == "Band-stop":
        b0 = 1.0
        b1 = -2.0 * cosw0
        b2 = 1.0
        a0 = 1.0 + alpha
        a1 = -2.0 * cosw0
        a2 = 1.0 - alpha
    elif filter_type == "Peak":
        A = 10.0 ** (gain_db / 40.0)
        b0 = 1.0 + alpha * A
        b1 = -2.0 * cosw0
        b2 = 1.0 - alpha * A
        a0 = 1.0 + alpha / A
        a1 = -2.0 * cosw0
        a2 = 1.0 - alpha / A
    elif filter_type == "Low Shelf":
        A = 10.0 ** (gain_db / 40.0)
        sq = 2.0 * math.sqrt(A) * alpha
        b0 = A * ((A + 1) - (A - 1) * cosw0 + sq)
        b1 = 2.0 * A * ((A - 1) - (A + 1) * cosw0)
        b2 = A * ((A + 1) - (A - 1) * cosw0 - sq)
        a0 = (A + 1) + (A - 1) * cosw0 + sq
        a1 = -2.0 * ((A - 1) + (A + 1) * cosw0)
        a2 = (A + 1) + (A - 1) * cosw0 - sq
    elif filter_type == "High Shelf":
        A = 10.0 ** (gain_db / 40.0)
        sq = 2.0 * math.sqrt(A) * alpha
        b0 = A * ((A + 1) + (A - 1) * cosw0 + sq)
        b1 = -2.0 * A * ((A - 1) + (A + 1) * cosw0)
        b2 = A * ((A + 1) + (A - 1) * cosw0 - sq)
        a0 = (A + 1) - (A - 1) * cosw0 + sq
        a1 = 2.0 * ((A - 1) - (A + 1) * cosw0)
        a2 = (A + 1) - (A - 1) * cosw0 - sq
    elif filter_type == "One-pole":
        bp = math.exp(-2.0 * math.pi * fc / sample_rate)
        return (1.0 - bp, 0.0, 0.0, -bp, 0.0)
    else:
        return (1.0, 0.0, 0.0, 0.0, 0.0)

    inv = 1.0 / a0
    return (b0 * inv, b1 * inv, b2 * inv, a1 * inv, a2 * inv)


def apply_biquad(samples, b0, b1, b2, a1, a2, state=None):
    z1, z2 = state if state else (0.0, 0.0)
    out = []
    for x in samples:
        y = b0 * x + z1
        z1 = b1 * x - a1 * y + z2
        z2 = b2 * x - a2 * y
        out.append(y)
    return out, (z1, z2)


def _fft_radix2(x):
    n = len(x)
    if n == 1:
        return x
    even = _fft_radix2(x[0::2])
    odd = _fft_radix2(x[1::2])
    out = [0] * n
    for k in range(n // 2):
        t = cmath.exp(-2j * cmath.pi * k / n) * odd[k]
        out[k] = even[k] + t
        out[k + n // 2] = even[k] - t
    return out


def compute_fft_db(samples, sample_rate):
    n = len(samples)
    nfft = 1
    while nfft < n:
        nfft <<= 1
    padded = [complex(s) for s in samples] + [0j] * (nfft - n)
    spectrum = _fft_radix2(padded)
    half = nfft // 2 + 1
    mags = [abs(spectrum[i]) / nfft for i in range(half)]
    ref = max(mags) if max(mags) > 0 else 1.0
    db = [20.0 * math.log10(max(m / ref, 1e-10)) for m in mags]
    freqs = [i * sample_rate / nfft for i in range(half)]
    return freqs, db

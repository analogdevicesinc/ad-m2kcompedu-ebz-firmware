"""Audio demo modules — worker thread and DSP helpers."""

from .worker import AudioWorker, AUDIO_BUF_SIZE, AUDIO_SAMPLE_RATE  # noqa: F401
from .dsp import (  # noqa: F401
    FILTER_TYPES,
    apply_biquad,
    biquad_coeffs,
    compute_fft_db,
)

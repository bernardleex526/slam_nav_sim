"""Time-grid sampling for independent evaluation data; no pose fitting or clock shifting."""

import math
import numpy as np


def timer_phase(timestamps, period=0.1):
    # Circular phase avoids the 0/period boundary and startup timestamp outliers.
    phase = np.mean(np.exp(2j * np.pi * (np.asarray(timestamps) % period) / period))
    return float(np.angle(phase) * period / (2 * np.pi))


def sample_reference(reference, sample_origin, max_gap_s, period=0.1, margin=0.2):
    if (
        len(reference) < 2
        or not np.isfinite(reference).all()
        or np.any(np.diff(reference[:, 0]) <= 0)
    ):
        raise ValueError("Independent reference must be finite and strictly time ordered")
    first = math.ceil((reference[0, 0] + margin - sample_origin) / period)
    last = math.floor((reference[-1, 0] - margin - sample_origin) / period)
    times = sample_origin + np.arange(first, last + 1) * period
    right = np.searchsorted(reference[:, 0], times)
    valid = (right > 0) & (right < len(reference))
    usable = np.flatnonzero(valid)
    gaps = reference[right[usable], 0] - reference[right[usable] - 1, 0]
    valid[usable] &= gaps <= max_gap_s
    accepted = times[valid]
    sampled = np.column_stack(
        (
            accepted,
            np.interp(accepted, reference[:, 0], reference[:, 1]),
            np.interp(accepted, reference[:, 0], reference[:, 2]),
            np.interp(accepted, reference[:, 0], np.unwrap(reference[:, 3])),
        )
    )
    return sampled, dict(
        method="linear interpolation of independent truth at estimator timer phase",
        sample_origin_s=float(sample_origin),
        sample_period_s=period,
        max_reference_gap_s=max_gap_s,
        expected_samples=len(times),
        available_samples=len(accepted),
        reference_availability=len(accepted) / len(times) if len(times) else 0.0,
        full_reference_interval_s=[
            float(reference[0, 0] + margin),
            float(reference[-1, 0] - margin),
        ],
        clock_offset_applied_s=0.0,
    )

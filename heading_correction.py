"""
heading_correction.py
Utility to correct raw AHRS heading based on calibration offsets.
The calibration provides offset values at cardinal points:
    0°   -> 274°
   90°   ->   5°
  180°   -> 100°
  270°   -> 189°
We linearly interpolate the offset between these points and apply it to the raw
heading. The result is wrapped to [0, 360) degrees.
"""
from typing import List, Tuple

# Calibration points (raw_angle, offset)
_CAL_POINTS: List[Tuple[float, float]] = [
    (0.0, 274.0),
    (90.0, 5.0),
    (180.0, 100.0),
    (270.0, 189.0),
    (360.0, 274.0),  # wrap around for interpolation continuity
]

def _interpolate_offset(angle: float) -> float:
    """Linearly interpolate the offset for a given raw angle.

    Args:
        angle: Raw heading in degrees (0‑360).
    Returns:
        Interpolated offset in degrees.
    """
    # Ensure angle is within [0, 360)
    angle = angle % 360.0
    # Find the segment that contains the angle
    for i in range(len(_CAL_POINTS) - 1):
        a0, o0 = _CAL_POINTS[i]
        a1, o1 = _CAL_POINTS[i + 1]
        if a0 <= angle <= a1:
            # Linear interpolation
            if a1 == a0:
                return o0
            t = (angle - a0) / (a1 - a0)
            return o0 + t * (o1 - o0)
    # Fallback (should never hit because of wrap point)
    return _CAL_POINTS[0][1]


def correct_heading(raw_angle: float) -> float:
    """Return the calibrated heading.

    The function adds the interpolated offset to the raw heading and wraps the
    result to the range [0, 360).
    """
    offset = _interpolate_offset(raw_angle)
    corrected = (raw_angle + offset) % 360.0
    return corrected

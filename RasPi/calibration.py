"""
Shared robot constants and the kinematic-angle -> servo-angle mapping.
Lives in its own module so gait.py, backends.py and robot_setup.py can all use it
without importing each other (robot_setup imports backends, so backends can't import robot_setup).
"""
import numpy as np

LENGTHS = np.array([111.9, 80, 152.4, 203.2])  # radius, coxa, femur, tibia

DEFAULT_CAL = [(90, -1, 0),  # J1 hip straight out at 90
               (90, -1, 0),  # J2 femur horizontal
               (0, -1, 0)]   # J3 knee

CALIBRATION = {leg: list(DEFAULT_CAL) for leg in range(1, 7)}


def thetas_to_servo(leg_id, thetas, clamp=False):
    """
    Kinematic joint angles (radians) -> servo degrees for one leg.

    Returns None if any angle is outside 0-180 (unless clamp=True, which limits
    each angle to 0-180 instead).
    """
    angles = [center + direction * np.degrees(theta) + trim
              for theta, (center, direction, trim) in zip(thetas, CALIBRATION[leg_id])]
    if clamp:
        return [min(180.0, max(0.0, a)) for a in angles]
    if any(a < 0 or a > 180 for a in angles):
        return None
    return angles
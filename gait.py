"""
The gait motions for the legs
"""

import numpy as np
from kinematics import Kinematics

LENGTHS = np.array([111.9, 80, 152.4, 203.2]) #radius, coxa, femur, tibia
kin = Kinematics(LENGTHS)

#step parameters
OFFSET  = 40 #how far behind home and how far in front (80 in total)
LIFT = 40 #how high the arc is
SWING_FRACTION = 0.5 #share of each cycle spent in the air (50%)
CYCLE_TIME = 2.0 #how long one full step takes
FORWARD = np.array([0.0, 1.0, 0.0]) #positive y is the front of the robot

#where each leg is in the cycle
#legs 1, 3, 5 start at top of arc
#legs 2, 4, 6 start on ground, groups are a half cycle apart
PHASE_OFFSETS = {1: 0.25, 2: 0.75, 3: 0.25, 4: 0.75, 5: 0.25, 6: 0.75}

DEFAULT_CAL = [(90, -1, 0), #J1 hip straight out at 90
               (90, -1, 0), #J2 femur horizontal
               (0, -1, 0)]  #J3 knee

CALIBRATION = {leg: list(DEFAULT_CAL) for leg in range(1, 7)}

HOME_BODY = kin.leg_xyz_home_in_body.astype(float)

def smoothen(u):
    """
    start slowly, speed up, slow down
    """
    return (1-np.cos(np.pi*u)) /2

def foot_in_body(leg_id, phase):
    """
    where the leg should be at a certain phase in the body frame
    """
    p = (phase + PHASE_OFFSETS[leg_id]) % 1.0
    home = HOME_BODY[leg_id -1]
    back = home- FORWARD * OFFSET
    front = home + FORWARD * OFFSET

    if p < SWING_FRACTION: 
        u = p / SWING_FRACTION
        lift = np.array([0.0, 0.0, LIFT *np.sin(np.pi*u)])
        return back + (front - back) * smoothen(u) + lift

    u = (p-SWING_FRACTION)/ (1-SWING_FRACTION)
    return front + (back-front)*u

def feet_at(phase):
    """
    all six feet at a phase in the body frame
    """
    return np.array([foot_in_body(leg,phase) for leg in range(1,7)])

def transition_feet(u):
    """
    moves the foot from home to the gait starting position
    """
    return HOME_BODY + (feet_at(0)-HOME_BODY) * smoothen(u)

def leg_servo_angles(leg_id, point_leg):
    """
    leg frame to servo degrees
    """
    thetas = kin.IK(*point_leg)
    if thetas is None:
        return None

    angles = [center + direction *np.degrees(theta) + trim 
              for theta, (center, direction, trim) in zip(thetas, CALIBRATION[leg_id])
              ]

    if any(a<0 or a>180 for a in angles):
        return None
    return angles

def servo_frame(feet_body, legs):
    """
    body frame to leg id
    """
    feet_leg = kin.feet_body_to_leg(feet_body)
    frame = {}
    for leg in legs:
        angles = leg_servo_angles(leg, feet_leg[leg-1])
        if angles is None:
            raise ValueError(f"Leg {leg} can't reach")
        frame[leg] = angles
    return frame

if __name__ == "__main__":
    for leg, angles in servo_frame(HOME_BODY, range(1, 7)).items():
        print(f"Leg {leg} home servo angles: {np.round(angles, 1)}")
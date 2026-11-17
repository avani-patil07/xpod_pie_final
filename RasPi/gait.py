"""
The gait motions for the legs
"""

import numpy as np
from calibration import CALIBRATION, LENGTHS, thetas_to_servo  # noqa: F401  (re-exported)

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

#directional walking (used by walk_feet)
MAX_YAW_STEP = np.radians(15) #how far the body turns per step at full yaw command
ACTIVITY_FULL = 0.2 #command size at which the foot lift reaches full height


def smoothen(u):
    """
    start slowly, speed up, slow down
    """
    return (1-np.cos(np.pi*u)) /2


class Gait:
    """
    Gait generator. Uses the Kinematics instance the Robot already has, so the
    link lengths / home pose are defined in exactly one place.
    """

    def __init__(self, kin):
        self.kin = kin
        self.home_body = kin.leg_xyz_home_in_body.astype(float)

    def foot_in_body(self, leg_id, phase, stride=None, lift_height=LIFT):
        """
        where the leg should be at a certain phase in the body frame
        stride: vector the foot travels during one step (default: straight forward, 2*OFFSET)
        """
        if stride is None:
            stride = FORWARD * 2 * OFFSET
        p = (phase + PHASE_OFFSETS[leg_id]) % 1.0
        home = self.home_body[leg_id -1]
        back = home- stride / 2
        front = home + stride / 2

        if p < SWING_FRACTION:
            u = p / SWING_FRACTION
            lift = np.array([0.0, 0.0, lift_height *np.sin(np.pi*u)])
            return back + (front - back) * smoothen(u) + lift

        u = (p-SWING_FRACTION)/ (1-SWING_FRACTION)
        return front + (back-front)*u

    def feet_at(self, phase, strides=None, lift_height=LIFT):
        """
        all six feet at a phase in the body frame
        strides: optional 6x3 array, one stride vector per leg
        """
        return np.array([self.foot_in_body(leg, phase,
                                           None if strides is None else strides[leg-1],
                                           lift_height)
                         for leg in range(1,7)])

    def command_strides(self, cmd):
        """
        cmd = (vx, vy, yaw) in the BODY frame (x right, y front, yaw counter-clockwise),
        each -1..1. Returns one stride vector per leg: the straight-line part is the same for
        every leg, the turning part is tangent to the circle around the body centre.
        """
        vx, vy, yaw = cmd
        longest = 2 * OFFSET
        lin = np.array([vx, vy, 0.0]) * longest
        strides = []
        for hx, hy, _ in self.home_body:
            s = lin + np.array([-hy, hx, 0.0]) * yaw * MAX_YAW_STEP
            n = np.linalg.norm(s)
            if n > longest:                 # keep every foot inside its reachable patch
                s *= longest / n
            strides.append(s)
        return np.array(strides)

    def walk_feet(self, phase, cmd):
        """
        feet for a walking command; cmd of zero gives the home pose
        """
        cmd = np.asarray(cmd, dtype=float)
        activity = min(1.0, np.max(np.abs(cmd)) / ACTIVITY_FULL) #no stepping on the spot when idle
        return self.feet_at(phase, self.command_strides(cmd), LIFT * activity)

    def transition_feet(self, u):
        """
        moves the foot from home to the gait starting position
        """
        return self.home_body + (self.feet_at(0)-self.home_body) * smoothen(u)

    def leg_servo_angles(self, leg_id, point_leg):
        """
        leg frame to servo degrees
        """
        thetas = self.kin.IK(*point_leg)
        if thetas is None:
            return None
        return thetas_to_servo(leg_id, thetas)

    def servo_frame(self, feet_body, legs):
        """
        body frame to leg id
        """
        feet_leg = self.kin.feet_body_to_leg(feet_body)
        frame = {}
        for leg in legs:
            angles = self.leg_servo_angles(leg, feet_leg[leg-1])
            if angles is None:
                raise ValueError(f"Leg {leg} can't reach")
            frame[leg] = angles
        return frame


if __name__ == "__main__":
    from kinematics import Kinematics
    gait = Gait(Kinematics(LENGTHS))
    for leg, angles in gait.servo_frame(gait.home_body, range(1, 7)).items():
        print(f"Leg {leg} home servo angles: {np.round(angles, 1)}")
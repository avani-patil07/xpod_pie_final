"""
Definition of the robot
"""
import math as m
import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import kinematics
from calibration import LENGTHS, thetas_to_servo

from backends import *

class Robot:
    def __init__(self, kin, backends=(), enforce_servo_limits=True):
        self.kin = kin
        self.backends = list(backends)
        self.enforce_servo_limits = enforce_servo_limits # reject poses a servo can't reach (0-180)
        self.feet = kin.leg_xyz_home_in_body.copy()      # body frame, 6x3
        self.angles = np.zeros((6, 3))                   # radians, kinematic zero

    def set_pose(self, roll=0, pitch=0, yaw=0):
        """Body rotates, feet stay planted at home."""
        return self.set_feet(self.kin.leg_xyz_home_in_body, rpy=(roll, pitch, yaw))

    def set_feet(self, feet_body, rpy=(0, 0, 0), xyz=(0, 0, 0)):
        """
        General entry point (used by the gait): six foot targets in the body frame,
        optionally with the body rotated / shifted. Returns False and changes nothing
        if any leg can't reach or a servo would leave 0-180.
        """
        leg_xyz = self.kin.feet_body_to_leg(feet_body, rpy, xyz)   # 6x3, leg frames
        angles = self.kin.IK_array(leg_xyz)
        if angles is None:
            return False                                 # reject whole pose, don't half-apply
        if self.enforce_servo_limits and any(
                thetas_to_servo(i + 1, a) is None for i, a in enumerate(angles)):
            return False
        self.feet = np.array(feet_body, dtype=float)
        self.angles = angles
        for b in self.backends:
            b.update(self)
        return True

    def close(self):
        for b in self.backends:
            if hasattr(b, "close"):
                b.close()


if __name__ == "__main__":
    lengths = LENGTHS
    my_kinematics = kinematics.Kinematics(lengths)
    my_plotter = PlotBackend(limits=[-300, 300])
    my_serial = SerialBackend(None, 115200, True)
    my_robot = Robot(my_kinematics, backends=[my_serial])

    for i in range(1, 1000):
        my_robot.set_pose(roll=0, pitch=0.0, yaw=1/3*np.sin(2*i / 180 * 2 * np.pi))
        # my_robot.set_pose(roll=0, pitch=0.0, yaw=0)
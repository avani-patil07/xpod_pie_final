"""
Definition of the robot
"""
import math as m
import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import kinematics

from backends import *


# class leg:
#     def __init__(self, id:int, rotation:int, lengths:tuple, base:np.array, current:np.array):
#         """
#         Args:
#             id (int): ID number of the leg. Refence external source for leg naming convention.
#             rotation (int): yaw rotation of the base coordinate frame
#             lengths (tuple): Lengths of all three legs in index order
#             base (np.array): coordinate of the base point of the leg
#             current (np.array): coordinate of the foot position 
#         """
#         self._id = id
#         self._rotation = rotation
#         self._lengths = lengths
#         self._base = base
#         self._current = current
#         self._angles = np.array([90, 90, 90])

#     @property
#     def id(self):
#         return self._id
#     @property
#     def rotation(self):
#         return self._rotation
#     @property
#     def base(self):
#         return self._base
#     @property
#     def current(self):
#         return self._current

#     @property
#     def joint_loc(self):
#         """
#         Locate joint locations based on current angles.
#         Return:
#             np.matrix of the current joint locations. [J1 (base); J2; J3; J4 (Foot)]
#         """
#         return None


#     def update_pos(self, new:np.array):
#         """
#         Move leg to new angles and position
#         Args:
#             new (np.array): new location of foot
#         Retruns:
#             True if successful. False if not.
#         """
#         if not self.possible(new):
#             return False
#         angles = IK(new)
#         if angles:
#             return False
#         self._angles = angles
#         self._current = new
#         return True

#     def possible(self, target:np.array):
#         if target.size != 3:
#             raise ValueError
#         x, y, z = target
#         if m.dist((0, 0, 0), (x, y, z)) < sum(self._lengths):
#             return True
#         return False

class Robot:
    def __init__(self, kin, backends=()):
        self.kin = kin
        self.backends = list(backends)
        self.feet = kin.leg_xyz_home_in_body.copy()      # body frame, 6x3
        self.angles = np.zeros((6, 3))                   # radians, kinematic zero

    def set_pose(self, roll=0, pitch=0, yaw=0):
        leg_xyz = self.kin.RPY_to_XYZ(roll, pitch, yaw)  # 6x3, leg frames
        angles = [self.kin.IK(*p) for p in leg_xyz]
        if any(a is None for a in angles):
            return False                                 # reject whole pose, don't half-apply
        self.angles = np.array(angles)
        for b in self.backends:
            b.update(self)
        return True


# class robot:
#     """
#     """
#     leg_gamma_deg = np.array([120, 60, 180, 0, 240, 300])
#     leg_gamma_rad = np.radians(leg_gamma_deg)
#     z_offset = 0
#     def __init__(self, r:float, lengths:tuple, my_serial:serial.Serial=None, debug:bool=False):
#         self.debug = debug
#         self._serial = my_serial
#         self._radius = r
#         self.legs = {x+1: leg(x+1, self.leg_gamma_rad[x], lengths, np.array([self._radius * np.cos(self.leg_gamma_rad[x]), self._radius * np.sin(self.leg_gamma_rad[x]), self.z_offset]), np.array([20, 0, -30])) for x in range(0, 6)}
#         if self.debug:
#             for id, leg0 in self.legs.items():
#                 print(f"Leg {id} located at [{round(leg0.base[0], 3)}, {round(leg0.base[1], 3)}, {round(leg0.base[2], 3)}]. Foot at [{round(leg0.current[0], 3)}, {round(leg0.current[1], 3)}, {round(leg0.current[2], 3)}]")
#         my_kinematics = kinematics.Kinematics()


#     def send_angles(self, ser, angles):
#         """
#         Send the angles the pi figures out to the Arduino in a string
#         Args:
#             ser: Serial port
#         """
#         # constrain angles
#         t1_unrounded = max(0, min(180, angles[0]))
#         t2_unrounded = max(0, min(180, angles[1]))
#         t3_unrounded = max(0, min(180, angles[2]))

#         # round angles
#         t1 = round(t1_unrounded, 1)
#         t2 = round(t2_unrounded, 1)
#         t3 = round(t3_unrounded, 1) 

#         # send to arduino
#         if self._serial:
#             ser.write(f"{t1},{t2},{t3}\n".encode())
#         if self.debug:
#             print(f"{t1},{t2},{t3}")

#     def init_plt(self):
#         plt.ion()
#         self.fig = plt.figure(figsize=(8, 6))
#         self.ax = self.fig.add_subplot(111, projection='3d')
#         self.ax.set_xlim([-1000, 1000])
#         self.ax.set_ylim([-1000, 1000])
#         self.ax.set_zlim([0, 2000])
#         self.ax.set_xlabel('X (mm)')
#         self.ax.set_ylabel('Y (mm)')
#         self.ax.set_zlabel('Z (mm)')

#     def draw_body(self):
#         """
#         Plot the body
#         """
#         body_artist


#     def plot_current(self):
#         """
#         Create a plot of the robot
#         """
#         self.plot_body()
#         for leg in self.legs.values():
#             self.draw_leg(leg.joint_loc)

if __name__ == "__main__":
    lengths = np.array([111.9, 80, 152.4, 203.2])
    my_kinematics = kinematics.Kinematics(lengths)
    my_plotter = PlotBackend(limits=[-300, 300])
    my_robot = Robot(my_kinematics, backends=[my_plotter])

    for i in range(1, 1000):
        my_robot.set_pose(roll=0, pitch=0.0, yaw=1/3*np.sin(2*i / 180 * 2 * np.pi))
        # my_robot.set_pose(roll=0, pitch=0.0, yaw=0)
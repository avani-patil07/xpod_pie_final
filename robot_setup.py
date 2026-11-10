"""
Definition of the robot
"""
import math as m
import numpy as np
from inverse_kinematics import inverse_kinematics as IK


class leg:
    def __init__(self, id:int, rotation:int, lengths:tuple, base:np.array, current:np.array):
        """
        Args:
            id (int): ID number of the leg. Refence external source for leg naming convention.
            rotation (int): yaw rotation of the base coordinate frame
            lengths (tuple): Lengths of all three legs in index order
            base (np.array): coordinate of the base point of the leg
            current (np.array): coordinate of the foot position 
        """
        self._id = id
        self._rotation = rotation
        self._lengths = lengths
        self._base = base
        self._current = current
        self._angles = np.array([90, 90, 90])

    @property
    def id(self):
        return self._id
    @property
    def rotation(self):
        return self.rotation
    @property
    def base(self):
        return self._base
    @property
    def current(self):
        return self._current

    def update_pos(self, new:np.array):
        """
        Move leg to new angles and position
        Args:
            new (np.array): new location of foot
        Retruns:
            True if successful. False if not.
        """
        angles = IK(new)
        if angles:
            return False
        self._angles = angles
        self._current = new
        return True


class robot:
    """
    """
    leg_gamma_deg = np.array([120, 60, 180, 0, 240, 300])
    leg_gamma_rad = np.radians(leg_gamma_deg)
    z_offset = 0
    def __init__(self, r:float, lengths:tuple):
        self._radius = r
        self.legs = {x+1: leg(x+1, lengths, np.array([self._radius * m.cos(self.leg_gamma_rad[x]), self._radius * m.sin(self.leg_gamma_rad[x]), self.z_offset]), np.array([20, 0, -30])) for x in range(0, 6)}
        for id, leg0 in self.legs.items():
            print(f"Leg {id} located at [{round(leg0.base[0], 3)}, {round(leg0.base[1], 3)}, {round(leg0.base[2], 3)}]. Foot at [{round(leg0.current[0], 3)}, {round(leg0.current[1], 3)}, {round(leg0.current[2], 3)}]")

    def transform_2_leg_frame(self, id:int, coordinates:np.array):
        """
        change any coordinate into the frame of a certain leg
        Args:
            id (int): ID number of leg
        Returns:
            np.array of the coordinates in the robot frame
        """
        leg = self.legs[id]
        gamma = leg.rotation
        translation = leg.base
        R_matrix = np.matrix([m.cos(gamma), -m.sin(gamma), 0], [m.sin(gamma), m.cos(gamma), 0], [0, 0, 1])
        final = R_matrix @ coordinates + translation
        return final

my_robot = robot(11.19, (8, 15.24, 20.32))
# my_robot = robot(5, (1, 1, 1))
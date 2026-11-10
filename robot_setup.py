"""
Definition of the robot
"""
import math as m
import numpy as np
from inverse_kinematics import inverse_kinematics as IK
import serial

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
        return self._rotation
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
        if not self.possible(new):
            return False
        angles = IK(new)
        if angles:
            return False
        self._angles = angles
        self._current = new
        return True

    def possible(self, target:np.array):
        if target.size != 3:
            raise ValueError
        x, y, z = target
        if m.dist((0, 0, 0), (x, y, z)) < sum(self._lengths):
            return True
        return False


class robot:
    """
    """
    leg_gamma_deg = np.array([120, 60, 180, 0, 240, 300])
    leg_gamma_rad = np.radians(leg_gamma_deg)
    z_offset = 0
    def __init__(self, r:float, lengths:tuple, my_serial:serial.Serial=None, debug:bool=False):
        self.debug = debug
        self._serial = my_serial
        self._radius = r
        self.legs = {x+1: leg(x+1, self.leg_gamma_rad[x], lengths, np.array([self._radius * np.cos(self.leg_gamma_rad[x]), self._radius * np.sin(self.leg_gamma_rad[x]), self.z_offset]), np.array([20, 0, -30])) for x in range(0, 6)}
        if self.debug:
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
        R_matrix = np.matrix([[np.cos(gamma), -np.sin(gamma), 0], [np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]])
        final = R_matrix.T @ coordinates - translation
        if self.debug:
            print(f"translation: {translation}\n Gamma: {gamma} \n R_matrix: {R_matrix} \nFinal: {final}")
        return final

    def send_angles(self, ser, angles):
        """
        Send the angles the pi figures out to the Arduino in a string
        Args:
            ser: Serial port
        """
        # constrain angles
        t1_unrounded = max(0, min(180, angles[0]))
        t2_unrounded = max(0, min(180, angles[1]))
        t3_unrounded = max(0, min(180, angles[2]))

        # round angles
        t1 = round(t1_unrounded, 1)
        t2 = round(t2_unrounded, 1)
        t3 = round(t3_unrounded, 1) 

        # send to arduino
        if self._serial:
            ser.write(f"{t1},{t2},{t3}\n".encode())
        if self.debug:
            print(f"{t1},{t2},{t3}")



# my_robot = robot(11.19, (8, 15.24, 20.32))
my_robot = robot(5, (1, 1, 1))
my_robot.transform_2_leg_frame(4, np.array([0, 0, 0]))
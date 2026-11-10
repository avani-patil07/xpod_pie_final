import numpy as np
from numpy import pi


class Kinematics:
    def __init__(self):
        """
        Body parameters
        
        beta: angle between each leg, in radians
        body_to_leg_offset: distance between body center point to leg base
        
        """
        self.beta = np.radians(60)
        self.body_to_leg_offset = 0
        
        """
        Leg link parameters, in mm

        L1: Coxa link
        L2: Femur link
        L3: Tibia link
        """
        self.L1 = 65
        self.L2 = 134
        self.L3 = 173

        """
        Default pose at rest
        """
        self.x_home = 0
        self.y_home = 0
        self.z_home = 0
 
        self.theta1_home, self.theta2_home, self.theta3_home = self.IK(self.x_home, self.y_home, self.z_home)

        # Apply home position to all legs
        self.xyz_home = np.array([self.x_home, self.y_home, self.z_home])

        """
        Body frame to leg frame rotational offsets

        Leg 3 = front left,  Leg 2 = front right
        
        Leg 4 = middle left, Leg 1 = middle right
        
        Leg 5 = back left,   Leg 6 = back right
        """

        # Angle needed to rotate leg frame to match body frame, in radians
        self.leg1_offset_angle = 0.0
        self.leg2_offset_angle = np.radians(-self.beta)
        self.leg3_offset_angle = np.radians(-(180-self.beta))
        self.leg4_offset_angle = np.radians(-180)
        self.leg5_offset_angle = np.radians(180-self.beta)
        self.leg6_offset_angle = np.radians(self.beta)

        self.offset_angles = np.array([
            self.leg1_offset_angle,
            self.leg2_offset_angle,
            self.leg3_offset_angle,
            self.leg4_offset_angle,
            self.leg5_offset_angle,
            self.leg6_offset_angle
        ])

        # Body frame -> Leg frame rotation matrices
        self.leg1_offset_rotation_matrix = self.get_leg_offset_rotation_matrix(self.leg1_offset_angle)
        self.leg2_offset_rotation_matrix = self.get_leg_offset_rotation_matrix(self.leg2_offset_angle)
        self.leg3_offset_rotation_matrix = self.get_leg_offset_rotation_matrix(self.leg3_offset_angle)
        self.leg4_offset_rotation_matrix = self.get_leg_offset_rotation_matrix(self.leg4_offset_angle)
        self.leg5_offset_rotation_matrix = self.get_leg_offset_rotation_matrix(self.leg5_offset_angle)
        self.leg6_offset_rotation_matrix = self.get_leg_offset_rotation_matrix(self.leg6_offset_angle)

        """
        Body frame to leg frame translational offsets
        """

        # Distance to translate leg frame to match body frame, in mm
        self.leg1_offset_x = 0
        self.leg2_offset_x = 0
        self.leg3_offset_x = 0
        self.leg4_offset_x = 0
        self.leg5_offset_x = 0
        self.leg6_offset_x = 0
        
        # Body frame -> Leg frame translation arrays (applied after rotation, so only considering x)
        self.leg1_offset_x_array = np.array([-self.body_to_leg_offset, 0, 0])
        self.leg2_offset_x_array = np.array([-self.body_to_leg_offset, 0, 0])
        self.leg3_offset_x_array = np.array([-self.body_to_leg_offset, 0, 0])
        self.leg4_offset_x_array = np.array([-self.body_to_leg_offset, 0, 0])
        self.leg5_offset_x_array = np.array([-self.body_to_leg_offset, 0, 0])
        self.leg6_offset_x_array = np.array([-self.body_to_leg_offset, 0, 0])

        # xyz home position of each leg in body frame
        self.leg1_xyz_home_in_body = np.array([(self.x_home + self.body_to_leg_offset), 0, self.z_home])
        self.leg2_xyz_home_in_body = np.array([(self.x_home + self.body_to_leg_offset)*np.cos(self.beta), (self.y_home + self.body_to_leg_offset)*np.sin(self.beta), self.z_home])
        self.leg3_xyz_home_in_body = np.array([-(self.x_home + self.body_to_leg_offset)*np.cos(self.beta), (self.y_home + self.body_to_leg_offset)*np.sin(self.beta), self.z_home])
        self.leg4_xyz_home_in_body = np.array([-(self.x_home + self.body_to_leg_offset), 0, self.z_home])
        self.leg5_xyz_home_in_body = np.array([-(self.x_home + self.body_to_leg_offset)*np.cos(self.beta), -(self.y_home + self.body_to_leg_offset)*np.sin(self.beta), self.z_home])
        self.leg6_xyz_home_in_body = np.array([(self.x_home + self.body_to_leg_offset)*np.cos(self.beta), -(self.y_home + self.body_to_leg_offset)*np.sin(self.beta), self.z_home])
        

    def get_leg_offset_rotation_matrix(self, offset_angle):
        """
        Generate rotation matrix to convert foot coordinates in the body frame to coordinates in the leg frames. 

        Parameters:
            offset_angle - angle needed to rotate leg frame to match body frame, in radians

        Returns:
            Rotation matrix to be applied to foot coordinates in body frame
        """
        
        return np.array([
            [np.cos(offset_angle), -np.sin(self.leg1_ang_offset), 0],
            [np.sin(self.leg1_ang_offset), np.cos(self.leg1_ang_offset), 0], 
            [0, 0, 1]
        ])


    def IK(self, x, y, z):
        """
        Calculate inverse kinematics on a single leg.

        Parameters:
            x, y, z - desired cartesian coordinate of foot tip, in mm

        Returns:
            theta1, theta2, theta3 - joint angles, in radians
        """

        # calculate radius from hip to foot
        r_horiz = np.sqrt(x**2 + y**2) - self.L1
        r = np.sqrt(r_horiz**2 + z**2)

        # calculate base joint angle, theta1
        theta1 = np.arctan(y/x)

        # calculate hip joint angle, theta2
        phi1 = np.arccos((r**2 + self.L2**2 - self.L3**2) / (2*r*self.L2))
        phi2 = np.arctan(z/r_horiz)
        theta2 = phi1 + phi2

        # calculate knee joint angle, theta3
        phi3 = np.arccos((self.L2**2 + self.L3**2 - r**2) / (2*self.L2*self.L3))
        theta3 = pi - phi3

        return theta1, theta2, theta3


    def IK_array(self, xyz_array):
        """
        Calculate inverse kinematics from array of foot coordinates for legs 1-N.

        Parameters: 
            xyz_array - foot coordinates in the form  [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN]]

        Returns:
            theta1_array - [theta1_1, theta1_2, theta1_3, ..., theta1_N]
            theta2_array - [theta2_1, theta2_2, theta2_3, ..., theta2_N]
            theta3_array - [theta3_1, theta3_2, theta3_3, ..., theta3_N]
        """
        theta1_array = np.array([])
        theta2_array = np.array([])
        theta3_array = np.array([])

        for x, y, z in xyz_array:
            theta1, theta2, theta3 = self.IK(x, y, z)
            theta1_array = np.append(theta1_array, theta1)
            theta2_array = np.append(theta2_array, theta2)
            theta3_array = np.append(theta3_array, theta3)

        return theta1_array, theta2_array, theta3_array


    def body_change_to_leg_frame(self, leg_id, destination_body_coord):
        """
        Convert change in body center position from body frame to leg frame.

        Parameters:
            leg_id - leg index (1-6)
            destination_body_coord - body center position

        Returns:
            Vector representing change in body center position in the selected leg frame.
        """

        body_to_leg_rotation = self.get_leg_offset_rotation_matrix(self.leg_offset_angles[leg_id])
        body_to_leg_translation = np.array([-self.body_to_leg_offset, 0, 0])

        body_start_leg_frame = np.matmul(body_to_leg_rotation, ) + body_to_leg_translation 
        body_end_leg_frame = np.matmul(body_to_leg_rotation, destination_body_coord) + body_to_leg_translation

        return body_start_leg_frame - body_end_leg_frame


    def body_to_foot_pos(self, body_coord):
        """
        Convert center point coordinate in body frame to foot coordinates in leg frame.

        Parameters:
            body_coord - center point coordinate in body frame

        Returns:
            Foot coordinates in the form  [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN]]
        """
        leg1_xyz = self.xyz_home - np.matmul(self.leg1_offset_rotation_matrix, body_coord)
        leg2_xyz = self.xyz_home - np.matmul(self.leg2_offset_rotation_matrix, body_coord)
        leg3_xyz = self.xyz_home - np.matmul(self.leg3_offset_rotation_matrix, body_coord)
        leg4_xyz = self.xyz_home - np.matmul(self.leg4_offset_rotation_matrix, body_coord)
        leg5_xyz = self.xyz_home - np.matmul(self.leg5_offset_rotation_matrix, body_coord)
        leg6_xyz = self.xyz_home - np.matmul(self.leg6_offset_rotation_matrix, body_coord)
        
        return np.array([leg1_xyz, leg2_xyz, leg3_xyz, leg4_xyz, leg5_xyz, leg6_xyz])


    def get_RPY_matrix(self, roll, pitch, yaw):
        """
        Generate roll, pitch, yaw matrix.

        Roll = rotation about x axis (banking, tilting side to side)
        Pitch = rotation about y axis (nodding, tilting up and down)
        Yaw = rotation about z axis (panning, twising side to side)

        Parameters:
            roll - roll angle, in radians
            pitch - pitch angle, in radians
            yaw - yaw angle, in radians

        Returns:
            3x3 cumulative rotation matrix, as numpy array
        """

        # Rotation matrix about x-axis (roll)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        # Rotation matrix about y-axis (pitch)
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        # Rotation matrix about z-axis (yaw)
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        return np.matmul(np.matmul(R_z, R_y), R_x)


    def RPY_to_XYZ(self, roll, pitch, yaw):
        """
        Convert roll, pitch, yaw of body in body frame to foot positions, starting from home pose.

        Parameters:
            roll - roll angle, in radians
            pitch - pitch angle, in radians
            yaw - yaw angle, in radians

        Returns:
            Foot coordinates in the form  [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN]]     
        """

        RPY_matrix = self.get_RPY_matrix(roll, pitch, yaw)

        body_to_leg1_rotation_matrix = np.matmul(self.leg1_offset_rotation_matrix, RPY_matrix)
        body_to_leg2_rotation_matrix = np.matmul(self.leg2_offset_rotation_matrix, RPY_matrix)
        body_to_leg3_rotation_matrix = np.matmul(self.leg3_offset_rotation_matrix, RPY_matrix)
        body_to_leg4_rotation_matrix = np.matmul(self.leg4_offset_rotation_matrix, RPY_matrix)
        body_to_leg5_rotation_matrix = np.matmul(self.leg5_offset_rotation_matrix, RPY_matrix)
        body_to_leg6_rotation_matrix = np.matmul(self.leg6_offset_rotation_matrix, RPY_matrix)

        leg1_xyz = np.matmul(body_to_leg1_rotation_matrix, self.leg1_xyz_home_in_body) + self.leg1_offset_x_array
        leg2_xyz = np.matmul(body_to_leg2_rotation_matrix, self.leg2_xyz_home_in_body) + self.leg2_offset_x_array
        leg3_xyz = np.matmul(body_to_leg3_rotation_matrix, self.leg3_xyz_home_in_body) + self.leg3_offset_x_array
        leg4_xyz = np.matmul(body_to_leg4_rotation_matrix, self.leg4_xyz_home_in_body) + self.leg4_offset_x_array
        leg5_xyz = np.matmul(body_to_leg5_rotation_matrix, self.leg5_xyz_home_in_body) + self.leg5_offset_x_array
        leg6_xyz = np.matmul(body_to_leg6_rotation_matrix, self.leg6_xyz_home_in_body) + self.leg6_offset_x_array
        
        return np.array([leg1_xyz, leg2_xyz, leg3_xyz, leg4_xyz, leg5_xyz, leg6_xyz])



if __name__ == "__main__":
    instance = hexapod()
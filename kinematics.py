import numpy as np
from numpy import pi


class Kinematics:
    """
    Leg 3 = front left,  Leg 2 = front right
    
    Leg 4 = middle left, Leg 1 = middle right
    
    Leg 5 = back left,   Leg 6 = back right
    """

    def __init__(self, lengths:np.array):
        
        """
        Leg link parameters, in mm
        
        L1: Coxa link
        L2: Femur link
        L3: Tibia link
        """
        self.L1 = lengths[1]
        self.L2 = lengths[2]
        self.L3 = lengths[3]

        
        """
        Body parameters

        beta: angle between adjacent leg frames, in degrees
        radius: distance between body center point to leg base, in mm
        C: body center point, i.e. the origin
        """
        self.beta = np.radians(60)
        self.radius = lengths[0]
        self.C = np.array([0, 0, 100])


        """
        Default pose at rest ("home" position)
        """
        self.x_home = 170
        self.y_home = 0
        self.z_home = -100

        home_angles = self.IK(self.x_home, self.y_home, self.z_home)
        if home_angles is None:
            raise ValueError("Home foot position is not reachable with these link lengths")
        self.theta1_home, self.theta2_home, self.theta3_home = home_angles
        self.home_angles = np.array(home_angles)
        self.xyz_home = np.array([self.x_home, self.y_home, self.z_home])

        # xyz home position of each leg in body frame
        self.leg1_xyz_home_in_body = np.array([ (self.x_home + self.radius), 0, self.z_home])
        self.leg2_xyz_home_in_body = np.array([ (self.x_home + self.radius) * np.cos(self.beta),  (self.x_home + self.radius) * np.sin(self.beta), self.z_home])
        self.leg3_xyz_home_in_body = np.array([-(self.x_home + self.radius) * np.cos(self.beta),  (self.x_home + self.radius) * np.sin(self.beta), self.z_home])
        self.leg4_xyz_home_in_body = np.array([-(self.x_home + self.radius), 0, self.z_home])
        self.leg5_xyz_home_in_body = np.array([-(self.x_home + self.radius) * np.cos(self.beta), -(self.x_home + self.radius) * np.sin(self.beta), self.z_home])
        self.leg6_xyz_home_in_body = np.array([ (self.x_home + self.radius) * np.cos(self.beta), -(self.x_home + self.radius) * np.sin(self.beta), self.z_home])

        self.leg_xyz_home_in_body = np.array([
            self.leg1_xyz_home_in_body,
            self.leg2_xyz_home_in_body,
            self.leg3_xyz_home_in_body,
            self.leg4_xyz_home_in_body,
            self.leg5_xyz_home_in_body,
            self.leg6_xyz_home_in_body
        ])


        """
        Body frame to leg frame rotational offsets
        """
        # Angle needed to rotate leg frame to match body frame, in radians
        self.leg1_offset_angle = 0
        self.leg2_offset_angle = -self.beta
        self.leg3_offset_angle = -(pi-self.beta)
        self.leg4_offset_angle = -pi
        self.leg5_offset_angle = pi-self.beta
        self.leg6_offset_angle = self.beta

        self.leg_offset_angles = np.array([
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


        self.leg_offset_rotation_matrices = np.array([
            self.leg1_offset_rotation_matrix,
            self.leg2_offset_rotation_matrix,
            self.leg3_offset_rotation_matrix,
            self.leg4_offset_rotation_matrix,
            self.leg5_offset_rotation_matrix,
            self.leg6_offset_rotation_matrix
        ])


        """
        Body frame to leg frame translational offsets
        """
        # Body frame -> Leg frame translation arrays (applied after rotation, so only considering x)
        self.leg1_offset_translation_vector = np.array([-self.radius, 0, 0])
        self.leg2_offset_translation_vector = np.array([-self.radius, 0, 0])
        self.leg3_offset_translation_vector = np.array([-self.radius, 0, 0])
        self.leg4_offset_translation_vector = np.array([-self.radius, 0, 0])
        self.leg5_offset_translation_vector = np.array([-self.radius, 0, 0])
        self.leg6_offset_translation_vector = np.array([-self.radius, 0, 0])

        self.leg_offset_translation_vectors = np.array([
            self.leg1_offset_translation_vector,
            self.leg2_offset_translation_vector,
            self.leg3_offset_translation_vector,
            self.leg4_offset_translation_vector,
            self.leg5_offset_translation_vector,
            self.leg6_offset_translation_vector
        ])
        

    def get_leg_offset_rotation_matrix(self, offset_angle):
        """
        Generate rotation matrix to convert foot coordinates in the body frame to coordinates in the leg frames. 

        Parameters:
            offset_angle - angle needed to rotate leg frame to match body frame, in radians

        Returns:
            Rotation matrix to be applied to foot coordinates in body frame
        """
        
        return np.array([
            [np.cos(offset_angle), -np.sin(offset_angle), 0],
            [np.sin(offset_angle), np.cos(offset_angle), 0], 
            [0, 0, 1]
        ])


    def FK(self, theta1, theta2, theta3):
        """
        Calculate forward kinematics on a single leg.

        Parameters:
            theta1, theta2, theta3 - joint angles, in radians

        Returns:
            x, y, z - foot tip coordinate, in mm
        """

        x = self.L1*np.cos(theta1) + (self.L2*np.cos(theta2) + self.L3*np.cos(theta2 + theta3))*np.cos(theta1)
        y = self.L1*np.sin(theta1) + (self.L2*np.cos(theta2) + self.L3*np.cos(theta2 + theta3))*np.sin(theta1)
        z = self.L2*np.sin(theta2) + self.L3*np.sin(theta2 + theta3) 

        return x, y, z

    def joint_positions(self, th1, th2, th3):
        """Hip, knee, foot positions in the leg frame (for plotting)."""
        hip  = np.array([self.L1*np.cos(th1), self.L1*np.sin(th1), 0])
        r2   = self.L2*np.cos(th2)
        knee = hip + np.array([r2*np.cos(th1), r2*np.sin(th1), self.L2*np.sin(th2)])
        foot = np.array(self.FK(th1, th2, th3))
        return np.array([[0, 0, 0], hip, knee, foot])

    def IK(self, x, y, z):
        """
        Calculate inverse kinematics on a single leg.

        Parameters:
            x, y, z - foot tip coordinate in the LEG frame, in mm

        Returns:
            (theta1, theta2, theta3) in radians, or None if the point is unreachable.
            theta3 is negative: the tibia bends DOWN relative to the femur, which is what
            FK() assumes. (It used to be positive, which put the foot above the hip, i.e.
            the robot appeared upside down.)
        """

        # calculate radius from hip to foot
        r_horiz = np.sqrt(x**2 + y**2) - self.L1
        r = np.sqrt(r_horiz**2 + z**2)
        if r < 1e-9:
            return None

        # law of cosines; if either argument leaves [-1, 1] the target is out of reach
        c_hip = (r**2 + self.L2**2 - self.L3**2) / (2*r*self.L2)
        c_knee = (self.L2**2 + self.L3**2 - r**2) / (2*self.L2*self.L3)
        if abs(c_hip) > 1 or abs(c_knee) > 1:
            return None

        # base joint angle, theta1
        theta1 = np.arctan2(y, x)

        # hip joint angle, theta2
        phi1 = np.arccos(c_hip)
        phi2 = np.arctan2(z, r_horiz)
        theta2 = phi1 + phi2

        # knee joint angle, theta3
        phi3 = np.arccos(c_knee)
        theta3 = phi3 - pi

        return theta1, theta2, theta3


    def IK_array(self, xyz_array):
        """
        Calculate inverse kinematics from array of foot coordinates for legs 1-N.

        Parameters: 
            xyz_array - foot coordinates in the form  [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN]]

        Returns:
            [[theta1_1, theta2__1, theta3_1], [theta1_2, theta2_2, theta3_2], [theta1_3, theta2_3, theta3_3], ..., [theta1_N, theta2_N, theta3_N]]
        """

        results = [self.IK(x, y, z) for x, y, z in xyz_array]
        if any(r is None for r in results):
            return None
        return np.array(results)


    def body_delta_to_leg_delta(self, leg_id, body_delta):
        """
        Convert change in body center position from body frame to leg frame.

        Parameters:
            leg_id - leg index (1-6)
            body_delta - vector [x, y, z] representing body center translation

        Returns:
            Vector representing change in body center position in the selected leg frame.
        """
        # convert leg_id (1-6) to array index (0-5)
        idx = leg_id - 1

        R = self.leg_offset_rotation_matrices[idx]
        t = self.leg_offset_translation_vectors[idx]

        return R @ body_delta + t

    def body_to_leg(self, leg_id, p_body):
        """Foot/point position in the body frame -> the same point in leg `leg_id`'s frame."""
        return self.body_delta_to_leg_delta(leg_id, p_body)

    def leg_to_body(self, leg_id, p_leg):
        """Inverse of body_delta_to_leg_delta."""
        i = leg_id - 1
        R, t = self.leg_offset_rotation_matrices[i], self.leg_offset_translation_vectors[i]
        return R.T @ (p_leg - t)


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


    def feet_body_to_leg(self, feet_body, rpy=(0, 0, 0), xyz=(0, 0, 0)):
        """
        Convert six foot targets (body frame, planted on the ground) into leg-frame
        coordinates for IK, optionally with the body rotated / shifted.

        Parameters:
            feet_body - 6x3 array of foot positions in the NEUTRAL body frame (ground-fixed)
            rpy       - (roll, pitch, yaw) of the body, radians
            xyz       - body translation, mm

        If the body rotates by R, ground-fixed points appear rotated by R^T in the body
        frame, hence the transpose (RPY_to_XYZ used R, which turned the feet the wrong way).
        """
        R = self.get_RPY_matrix(*rpy)
        view = (np.asarray(feet_body, dtype=float) - np.asarray(xyz, dtype=float)) @ R  # = R.T @ p for each row
        return np.array([self.body_to_leg(i + 1, p) for i, p in enumerate(view)])


    def RPY_to_XYZ(self, roll, pitch, yaw):
        """
        Convert roll, pitch, yaw of body in body frame to foot positions in leg frame,
        starting from home pose (feet stay planted, body tilts).

        Returns:
            Foot coordinates in the form  [[x1,y1,z1], ..., [x6,y6,z6]]
        """
        return self.feet_body_to_leg(self.leg_xyz_home_in_body, (roll, pitch, yaw))



if __name__ == "__main__":
    # quick self-test: FK(IK(p)) should return p
    k = Kinematics(np.array([111.9, 80, 152.4, 203.2]))
    p = np.array([170.0, 20.0, -100.0])
    print(p, "->", np.round(k.FK(*k.IK(*p)), 3))
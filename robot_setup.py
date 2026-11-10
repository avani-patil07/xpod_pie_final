"""
Definition of the robot
"""
import math
import numpy as np
from inverse_kinematics import inverse_kinematics as IK


class leg:
    def __init__(self, lengths:tuple, base:np.array, current:np.array):
        """
        Args:
            lengths (tuple): Lengths of all three legs in index order
            base (np.array): coordinate of the base point of the leg
            current (np.array): coordinate of the foot position 
        """
        self._lengths = lengths
        self._base = base
        self._current = current
        self._angles = np.array([90, 90, 90])

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
    def __init__(self):
        joint_num_pairs = ["00"] + [str(x) + str(y) for x in range(1, 7) for y in range(0, 4)]
        self.joint_names = ["j" + x for x in joint_num_pairs]
        self.joints = {x: joint(self.joint_names(x)) for x in len(self.joint_names)}
        
        self.link_names = [ "l0010", "l1011", "l1112", "l1213", 
                            "l0020", "l2021", "l2122", "l2223", 
                            "l0030", "l3031", "l3132", "l3233", 
                            "l0040", "l4041", "l4142", "l4243", 
                            "l0050", "l5051", "l5152", "l5253", 
                            "l0060", "l6061", "l6162", "l6263"]
        for name in self.link_names:
            start_name = f"j{name[1:3]}"
            end_name   = f"j{name[3:5]}"
            start_joint = self.joints.get(f"j{start_name}")
            end_joint = self.joints.get(f"j{end_name}")
            length = math.dist(start_joint.loc, end_joint.loc)
            if start_joint and end_joint:
                self.links = {name: link(name, start_joint, end_joint, length)}
        print(self.links)

        

my_robot = robot()
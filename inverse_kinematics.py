"""
Inverse Kinematics copied from the Arduino code
"""
import math

L1 = 65   # coxa length (mm)
L2 = 152.4  # femur length (mm)
L3 = 203.2  # tibia length (mm)

def inverse_kinematics(x, y, z):
    """
    Returns angles relative to the servo (0-180) as (theta1, theta2, theta3),
    or None if the target is out of reach.
    """
    # angle between x-axis and end point, mapped by the center of the servo
    theta1 = 90 - math.degrees(math.atan2(z, x))

    # distance from J1 to the end point, subtracting L1 to know how far L2 and L3 must reach
    r_horiz = math.sqrt(x*x + z*z) - L1
    # from J1 to the end point
    r = math.sqrt(r_horiz*r_horiz + y*y)

    # Check whether the target is within the leg's reach
    if r < abs(L2 - L3) or r > L2 + L3:
        return None

    phi2 = math.atan2(y, r_horiz)
    phi1 = math.acos((r*r + L2*L2 - L3*L3) / (2*r*L2))
    phi3 = math.acos((L2*L2 + L3*L3 - r*r) / (2*L2*L3))

    theta2 = 90 - math.degrees(phi1 + phi2)
    theta3 = 180 - math.degrees(phi3)

    return theta1, theta2, theta3
"""
Gets one leg to move in an arc
"""
import math
import time
from inverse_kinematics import inverse_kinematics
import serial
import numpy as np

OFFSET = 40         # how far we want the foot to drag back / forwards
LIFT = 80           # highest point of the arc
SWING_STEPS = 30    # points along the arc (change for smoothness)
DRAG_STEPS = 20     # points along the drag
STEP_DELAY = 0.05   # sec between points (smaller = faster)

STARTING = np.array([200, -80,0]) #starting position of leg

PORT = 'COM3'
BAUD = 115200

def send_angles(ser, angles):
    """
    Send the angles the pi figures out to the Arduino in a string
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
    ser.write(f"{t1},{t2},{t3}\n".encode())

def move_to(ser, point):
    """
    which point to move to, false if can't reach
    """
    angles = inverse_kinematics(*point)

    if angles is None:
        return False
    send_angles(ser, angles)
    return True

def drag_to(ser, start, target):
    """
    Slide the foot on the ground
    """
    for i in range(1, DRAG_STEPS+1):
        t = i/ DRAG_STEPS
        point = start + (target-start) *t #moves x,y,z the same fraction
        move_to(ser, point)
        time.sleep(STEP_DELAY)

    return target

def arc_to(ser, start, target):
    """
    Lift foot and carry it along the arc
    """
    for i in range(1, SWING_STEPS+1):
        t = i/ SWING_STEPS
        s = (1-math.cos(math.pi*t))/ 2 #start slowly, speed up, slow down

        lift = np.array([0, LIFT * math.sin(math.pi * t), 0])
        point = start + (target-start) * s + lift
        move_to(ser, point)
        time.sleep(STEP_DELAY)
    return target

def step_cycle(ser, home):
    """
    Drag by OFFSET, arc forwards by twice that, then drag backwards
    """
    shift = np.array([0,0,OFFSET])
    back = home - shift
    front = home + shift

    for point in (back, front):
        if inverse_kinematics(*point) is None:
            print("cannot reach")
            return home #do not change if point is unreachable

    foot = drag_to(ser, home, back) #drags back
    foot = arc_to(ser, foot, front)
    foot = drag_to(ser, foot, home)
    return foot

def main():
    """
    runs the main movement
    """
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    move_to(ser, STARTING)
    time.sleep(1)

    step_cycle(ser, STARTING)
    ser.close()

if __name__ == "__main__":
    main()
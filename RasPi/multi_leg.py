"""
Runs the gait on every leg, through the Robot + backends pipeline
"""
import time
import numpy as np
from RasPi.kinematics import Kinematics
from RasPi.calibration import LENGTHS
from RasPi.gait import Gait, CYCLE_TIME
from RasPi.robot_setup import Robot
from RasPi.backends import SerialBackend, PlotBackend

PORT = 'COM3'
BAUD = 115200
USE_PLOT = True #live 3D view; drawing is slow, so leave off for real-time hardware runs

CYCLES = 3 #how many steps to take
TICK = 0.02 #seconds between updates 50 per second
TRANSITION_TIME = 1.0 #seconds to go in between standing and gait start

def send(robot, feet_body):
    """
    send the coordinates for the legs (IK -> robot.angles -> every backend)
    """
    if not robot.set_feet(feet_body):
        raise ValueError("Pose rejected: unreachable or outside servo limits")

def run_timed(robot, duration, feet_function):
    """
    call feet_function over a duration
    """
    start = time.monotonic()
    tick = 0
    while True:
        u = min((time.monotonic()-start) / duration, 1.0) #based on time elapsed
        send(robot, feet_function(u))
        if u >= 1.0:
            break
        tick += 1
        time.sleep(max(0, start + tick * TICK - time.monotonic())) #wait for next time tick

def main():
    kin = Kinematics(LENGTHS)
    gait = Gait(kin)

    backends = [SerialBackend(PORT, BAUD)]
    if USE_PLOT:
        backends.append(PlotBackend())
    robot = Robot(kin, backends=backends)

    def walk_feet(u):
        """
        turn u into gait phase
        """
        return gait.feet_at(u*CYCLES)

    def return_home_feet(u):
        """
        from pose to home
        """
        return gait.transition_feet(1-u)

    try:
        send(robot, gait.home_body)
        time.sleep(1)

        run_timed(robot, TRANSITION_TIME, gait.transition_feet) #home to start
        run_timed(robot, CYCLES * CYCLE_TIME, walk_feet) #walk
        run_timed(robot, TRANSITION_TIME, return_home_feet) #from start to home
    finally:
        robot.close()

if __name__ == "__main__":
    main()
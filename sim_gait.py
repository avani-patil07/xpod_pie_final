import time
from kinematics import Kinematics
from calibration import LENGTHS
from gait import Gait, CYCLE_TIME
from robot_setup import Robot
from backends import PlotBackend, SerialBackend

kin = Kinematics(LENGTHS)
gait = Gait(kin)
robot = Robot(kin, backends=[PlotBackend(), SerialBackend(None, 115200, debug=False)])

CYCLES = 6
start = time.monotonic()
while (u := (time.monotonic() - start) / (CYCLES * CYCLE_TIME)) < 1:
    robot.set_feet(gait.feet_at(u * CYCLES))
import time
from RasPi.kinematics import Kinematics
from RasPi.calibration import LENGTHS
from RasPi.gait import Gait, CYCLE_TIME
from RasPi.robot_setup import Robot
from RasPi.backends import PlotBackend, SerialBackend

kin = Kinematics(LENGTHS)
gait = Gait(kin)
robot = Robot(kin, backends=[PlotBackend(), SerialBackend(None, 115200, debug=False)])

CYCLES = 6
start = time.monotonic()
while (u := (time.monotonic() - start) / (CYCLES * CYCLE_TIME)) < 1:
    robot.set_feet(gait.feet_at(u * CYCLES))
"""
Definition of the robot
"""
import math as m
import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import threading
import time
import kinematics
from calibration import LENGTHS, thetas_to_servo
from gait import Gait, CYCLE_TIME
from wifi_communication import UDPNode

from backends import *

class Robot:
    def __init__(self, kin, backends=(), enforce_servo_limits=True):
        self.kin = kin
        self.backends = list(backends)
        self.enforce_servo_limits = enforce_servo_limits # reject poses a servo can't reach (0-180)
        self.feet = kin.leg_xyz_home_in_body.copy()      # body frame, 6x3
        self.angles = np.zeros((6, 3))                   # radians, kinematic zero

    def set_pose(self, roll=0, pitch=0, yaw=0):
        """Body rotates, feet stay planted at home."""
        return self.set_feet(self.kin.leg_xyz_home_in_body, rpy=(roll, pitch, yaw))

    def set_feet(self, feet_body, rpy=(0, 0, 0), xyz=(0, 0, 0)):
        """
        General entry point (used by the gait): six foot targets in the body frame,
        optionally with the body rotated / shifted. Returns False and changes nothing
        if any leg can't reach or a servo would leave 0-180.
        """
        leg_xyz = self.kin.feet_body_to_leg(feet_body, rpy, xyz)   # 6x3, leg frames
        angles = self.kin.IK_array(leg_xyz)
        if angles is None:
            return False                                 # reject whole pose, don't half-apply
        if self.enforce_servo_limits and any(
                thetas_to_servo(i + 1, a) is None for i, a in enumerate(angles)):
            return False
        self.feet = np.array(feet_body, dtype=float)
        self.angles = angles
        for b in self.backends:
            b.update(self)
        return True

    def close(self):
        for b in self.backends:
            if hasattr(b, "close"):
                b.close()


# ---------------------------------------------------------------------------
# Remote control: [x, y, yaw] arrives over UDP from teleop.py on the laptop
# ---------------------------------------------------------------------------
TICK = 0.02          # seconds between updates (50 per second)
CMD_TIMEOUT = 0.5    # no packet for this long -> robot stops (failsafe)
CMD_SLEW = 3.0       # max change of the command per second (stops the feet jumping)


class CommandLink:
    """Latest [x, y, yaw] received over UDP, safe to read from the main loop."""

    def __init__(self):
        self._lock = threading.Lock()
        self._cmd = np.zeros(3)
        self._stamp = -np.inf

    def on_message(self, text, addr):
        # packets look like "teleop: 0.500,0.000,-0.200" (UDPNode puts the sender name in front)
        try:
            x, y, yaw = (float(v) for v in text.split(":", 1)[1].split(","))
        except (IndexError, ValueError):
            return                       # ignore anything that isn't a command
        with self._lock:
            self._cmd = np.array([x, y, yaw])
            self._stamp = time.monotonic()

    def latest(self):
        """(command, seconds since it arrived)"""
        with self._lock:
            return self._cmd.copy(), time.monotonic() - self._stamp


def controller_to_body(cmd):
    """
    Controller frame (x forward, y left, yaw counter-clockwise)
    -> robot body frame (x right, y front, yaw counter-clockwise).
    """
    x, y, yaw = np.clip(cmd, -1, 1)
    lin = np.hypot(x, y)
    if lin > 1:                          # diagonal on a keyboard would be 1.41
        x, y = x / lin, y / lin
    return np.array([-y, x, yaw])


def drive_from_udp(robot, broadcast_ip='255.255.255.255', send_port=5002, listen_port=5001):
    """Walk the robot from UDP commands until Ctrl+C."""
    gait = Gait(robot.kin)
    link = CommandLink()
    node = UDPNode(broadcast_ip, send_port, listen_port, "robot", on_message=link.on_message)
    node.start_listening()

    robot.set_feet(gait.home_body)
    node.broadcast("ready")
    print(f"Listening for commands on UDP port {listen_port}. Ctrl+C to stop.")

    cmd = np.zeros(3)            # smoothed command actually driving the gait
    phase = 0.0
    last_warn = 0.0
    last = next_tick = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            dt, last = now - last, now

            target, age = link.latest()
            target = np.zeros(3) if age > CMD_TIMEOUT else controller_to_body(target)
            step = CMD_SLEW * dt
            cmd = cmd + np.clip(target - cmd, -step, step)

            phase = (phase + dt / CYCLE_TIME) % 1.0
            if not robot.set_feet(gait.walk_feet(phase, cmd)) and now - last_warn > 1.0:
                last_warn = now
                print("Pose rejected (unreachable / servo limit) - holding last pose")
                node.broadcast("pose rejected")

            next_tick += TICK
            time.sleep(max(0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        robot.set_feet(gait.home_body)
        robot.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hexapod: remote control over UDP (default) or yaw demo")
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial port of the servo controller")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--plot", action="store_true", help="also show the live 3D plot (needs a display)")
    parser.add_argument("--debug", action="store_true", help="print every serial frame that is sent")
    parser.add_argument("--ip", default=None, help="where status replies go (default: 127.0.0.1 with --sim, else broadcast)")
    parser.add_argument("--sim", action="store_true", help="no serial port: show the live 3D plot only (local testing)")
    parser.add_argument("--demo", action="store_true", help="original yaw-wobble demo (no serial, prints only)")
    args = parser.parse_args()

    lengths = LENGTHS
    my_kinematics = kinematics.Kinematics(lengths)

    if args.demo:
        my_plotter = PlotBackend(limits=[-300, 300])
        my_serial = SerialBackend(None, 115200, True)
        my_robot = Robot(my_kinematics, backends=[my_serial])

        for i in range(1, 1000):
            my_robot.set_pose(roll=0, pitch=0.0, yaw=1/3*np.sin(2*i / 180 * 2 * np.pi))
            # my_robot.set_pose(roll=0, pitch=0.0, yaw=0)
    else:
        backends = [] if args.sim else [SerialBackend(args.port, args.baud, args.debug)]
        if args.plot or args.sim:
            backends.append(PlotBackend(limits=[-300, 300]))
        # for same-machine testing use --ip 127.0.0.1 (--sim does this automatically)
        reply_ip = args.ip or ("127.0.0.1" if args.sim else "255.255.255.255")
        drive_from_udp(Robot(my_kinematics, backends=backends), broadcast_ip=reply_ip)
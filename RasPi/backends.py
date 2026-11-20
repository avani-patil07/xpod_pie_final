import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import serial
import time
from calibration import thetas_to_servo

class PlotBackend:
    """Live 3D view in the body frame: the hexagonal body stays put, the legs move."""

    # tripod A (legs 1,3,5) and tripod B (legs 2,4,6) get different colours,
    # which makes the gait easy to see
    LEG_COLORS = {1: "tab:blue", 3: "tab:blue", 5: "tab:blue",
                  2: "tab:orange", 4: "tab:orange", 6: "tab:orange"}

    def __init__(self, limits=(-300, 300), pause=0.001):
        self.pause = pause
        plt.ion()
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # z range is symmetric too: the feet hang BELOW the body (z < 0)
        self.ax.set_xlim(limits)
        self.ax.set_ylim(limits)
        self.ax.set_zlim(limits)
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.set_xlabel('X (mm)  right')
        self.ax.set_ylabel('Y (mm)  front')
        self.ax.set_zlabel('Z (mm)  up')

        # Create Line3D artists per leg (leg_id -> Line3D)
        self.leg_lines = {}
        for leg_id in range(1, 7):
            line, = self.ax.plot([], [], [], 'o-', lw=2, markersize=5,
                                 color=self.LEG_COLORS[leg_id], label=f"leg {leg_id}")
            self.leg_lines[leg_id] = line
        self.ax.legend(loc="upper left", fontsize=7)

        self.body_artists = None   # hexagon is drawn on the first update (needs the kinematics)

    def _draw_body(self, kin):
        """Hexagon through the six leg base joints (leg 1 -> 2 -> ... -> 6 -> 1)."""
        bases = np.array([kin.leg_to_body(leg_id, np.zeros(3)) for leg_id in range(1, 7)])
        closed = np.vstack([bases, bases[:1]])

        outline, = self.ax.plot(closed[:, 0], closed[:, 1], closed[:, 2], 'k-', lw=3)
        face = Poly3DCollection([bases], alpha=0.25, facecolor="grey", edgecolor="none")
        self.ax.add_collection3d(face)

        # mark the front of the robot (+y, between legs 2 and 3)
        front = 0.5 * (bases[1] + bases[2])
        arrow, = self.ax.plot([front[0], front[0]], [front[1], front[1] + 40],
                              [front[2], front[2]], 'r-', lw=3)
        label = self.ax.text(front[0], front[1] + 50, front[2], "FRONT", color="red", ha="center")
        self.body_artists = (outline, face, arrow, label)

    def update(self, robot):
        if self.body_artists is None:
            self._draw_body(robot.kin)

        for leg_id in range(1, 7):
            # robot.angles is indexed 0 to 5, leg_id is 1 to 6
            pts = robot.kin.joint_positions(*robot.angles[leg_id-1])
            pts_body = np.array([robot.kin.leg_to_body(leg_id, p) for p in pts])

            line = self.leg_lines[leg_id]
            line.set_data(pts_body[:, 0], pts_body[:, 1])
            line.set_3d_properties(pts_body[:, 2])

        # Refresh canvas and pause briefly to update the GUI event loop
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(self.pause)

    def close(self):
        plt.ioff()


class SerialBackend:
    def __init__(self, port:str, baud_rate:int, debug:bool=False, legacy_format:bool=False):
        self.debug = debug
        self.legacy_format = legacy_format   # old "a,b,c;a,b,c;...", otherwise "L1:a,b,c;L2:..."
        self._port = port
        self._baud_rate = baud_rate
        self._ser = serial.Serial(self._port, self._baud_rate, timeout=1, write_timeout=0.1) if port else None
        if self._ser:
            time.sleep(2)  # Wait for microcontroller to reset and initialize

    @property
    def port(self):
        return self._port

    @property
    def baud_rate(self):
        return self._baud_rate

    def update(self, robot):
        leg_count, _ = robot.angles.shape
        parts = []
        for i in range(0, leg_count):
            # kinematic radians -> calibrated servo degrees (clamped to 0-180)
            servo = thetas_to_servo(i + 1, robot.angles[i], clamp=True)
            joint_values = ",".join(f"{a:.1f}" for a in servo)
            parts.append(joint_values if self.legacy_format else f"L{i + 1}:{joint_values}")
        message = ";".join(parts) + (";" if self.legacy_format else "") + "\n"
        if self._ser:
            try:
                self._ser.write(message.encode())
            except serial.SerialTimeoutException:
                self._ser.reset_output_buffer()   # drop the stale frame, keep going
                print("Serial write timed out - frame dropped")
        if self.debug:
            print(f"Sent: {message}", end="")

    def close(self):
        if self._ser:
            self._ser.close()
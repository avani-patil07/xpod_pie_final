import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import serial
import time

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
    def __init__(self, port:str, baud_rate:int, debug:bool=False):
        self.debug = debug
        self._port = port
        self._baud_rate = baud_rate
        self._ser = serial.Serial(self._port, self._baud_rate, timeout=1) if port else None
        time.sleep(2)  # Wait for microcontroller to reset and initialize

    @property
    def port(self):
        return self._port

    @property
    def baud_rate(self):
        return self._baud_rate
    
    def update(self, robot):
        message = ''
        leg_count, _ = robot.angles.shape
        for i in range(0, leg_count):
            joint_values = list(str(round(np.degrees(x), 2)) for x in robot.angles[i].tolist())
            message += ",".join(joint_values) + ";"
        message += '\n'
        # self._ser.write(f"{message}\n".encode())
        if self.debug:
            print(f"Sent: {message}")
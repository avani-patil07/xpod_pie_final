"""
Runs on the LAPTOP: reads keyboard / joystick (controller.py) and streams [x, y, yaw] to the Pi over UDP.
"""
import argparse
import time
import pygame
from controller import controller
from wifi_communication import UDPNode

# 255.255.255.255 reaches every device on the network. If it doesn't (VPN, several adapters),
# use your subnet's broadcast address (e.g. 192.168.1.255) or the Pi's own IP.
TARGET_IP = '192.168.16.148'
SEND_PORT = 5001      # the Pi listens here
LISTEN_PORT = 5002    # the Pi's status messages arrive here
RATE_HZ = 50


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=TARGET_IP, help="where to send (127.0.0.1 for local testing)")
    target_ip = parser.parse_args().ip

    node = UDPNode(target_ip, SEND_PORT, LISTEN_PORT, "teleop",
                   on_message=lambda text, addr: print(f"\n[robot] {text}"))
    node.start_listening()

    ctl = controller()
    print(f"Input mode: {ctl.mode}. Sending to {target_ip}:{SEND_PORT}. Ctrl+C to quit.")
    if ctl.mode == "keyboard":
        print("w/s = forward/back, a/d = left/right, q/e = turn left/right")

    try:
        while True:
            x, y, yaw = ctl.get_input()
            node.broadcast(f"{x:.3f},{y:.3f},{yaw:.3f}")
            time.sleep(1 / RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):                       # UDP can drop packets: make sure the robot hears "stop"
            node.broadcast("0.000,0.000,0.000")
            time.sleep(0.02)
        pygame.quit()


if __name__ == "__main__":
    main()
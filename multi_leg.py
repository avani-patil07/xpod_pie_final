"""
Runs the gait on every leg
"""
import time
import serial
import gait

PORT = 'COM3'
BAUD = 115200

LEGS = [1, 2, 3, 4, 5, 6]
CYCLES = 3 #how many steps to take
TICK = 0.02 #seconds between updates 50 per second
TRANSITION_TIME = 1.0 #seconds to go in between standing and gait start

def send(ser, feet_body):
    """
    send the coordinates for the legs
    """
    frame = gait.servo_frame(feet_body, LEGS)
    parts = [f"L{leg}:{a[0]:.1f},{a[1]:.1f},{a[2]:.1f}" for leg, a in frame.items()]
    ser.write((";".join(parts) + "\n").encode())

def run_timed(ser, duration, feet_function):
    """
    call feet_function over a duration
    """
    start = time.monotonic()
    tick = 0
    while True:
        u = min((time.monotonic()-start) / duration, 1.0) #based on time elapsed
        send(ser, feet_function(u))
        if u >= 1.0:
            break
        tick += 1
        time.sleep(max(0, start + tick * TICK - time.monotonic())) #wait for next time tick

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

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    try:
        send(ser, gait.HOME_BODY)
        time.sleep(1)

        run_timed(ser, TRANSITION_TIME, gait.transition_feet) #home to start
        run_timed(ser, CYCLES * gait.CYCLE_TIME, walk_feet) #walk
        run_timed(ser, TRANSITION_TIME, return_home_feet) #from start to home
    finally:
        ser.close()

if __name__ == "__main__":
    main()
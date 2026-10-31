import pygame
import sys
import time
import keyboard

class controller():
    """
    Class for taking input from the user
    """
    keyboard_associations = {"w":"+x", "a": "+y", "s":"-x", "d":"-y", "q":"+yaw", "e": "-yaw"}
    keyboard_max = 0.5

    def __init__(self, debug=False):
        """
        Initialize the controller. If no joystick found, default to keyboard.
        """
        self._debug = debug
        joystick_count = pygame.joystick.get_count()
        self.mode = "joystick"
        if joystick_count == 0:
            if self._debug == True:
                print("Error: No joystick detected. Defaulting to keyboard mode")
            self.mode = "keyboard"
        else:
            if self._debug == True:
                print("Joystick detected. Continuing in joystick mode")
            # Initialize pygame and its joystick module
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

    def get_input(self):
        pygame.event.pump()
        if self.mode == "joystick":
            input = self._get_joystick()
        else:
            input = self._get_keyboard()

        return input

    def _get_joystick(self):
        """Get current inputs from controller"""
        # Updates internal states
        pygame.event.pump()

        # Read different part of controller
        throttle = (- self.joystick.get_axis(3) + 1) / 2
        x = - self.joystick.get_axis(1) * throttle
        y = - self.joystick.get_axis(0) * throttle
        yaw = - self.joystick.get_axis(2) * throttle

        # Print the data to console
        if self._debug == True:
            print(f"x: {x:6.2f} | y: {y:6.2f} | CW-CCW: {yaw:6.2f} | Scalar: {throttle:6.2f}", end="\r")
        return [x, y, yaw]

    def _get_keyboard(self):
        """
        Get current inputs from keyboard based on key associations.
        Returns a dictionary mapping the action string to a boolean state (True if pressed).
        """
        # Efficiently sample the state of only the keys defined in your map
        inputs = {
            action: int(keyboard.is_pressed(key)) * self.keyboard_max
            for key, action in self.keyboard_associations.items()
        }
        x = inputs["+x"] - inputs["-x"]
        y = inputs["+y"] - inputs["-y"]
        yaw = inputs["+yaw"] - inputs["-yaw"]
        
        if self._debug == True:
            print(f"x: {x:6.2f} | y: {y:6.2f} | CW-CCW: {yaw:6.2f} | Scalar: {self.keyboard_max}", end="\r")

        return [x, y, yaw]



if __name__ == "__main__":
    robot_controller = controller()
    try:
        while True:
            # Sleep briefly to avoid maxing out your CPU
            results = robot_controller.get_input()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nExiting script.")
        pygame.quit()

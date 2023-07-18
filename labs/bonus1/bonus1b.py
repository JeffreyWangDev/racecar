"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Bonus 1B - IMU: Driving in Shapes
"""

################################################################################
# Imports
################################################################################

import sys

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

import cv2 as cv
import numpy as np
import math


################################################################################
# Global variables
################################################################################

rc = racecar_core.create_racecar()

################################################################################
# Functions
################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Bonus Lab 1B - IMU: Driving in Shapes\n"
        "\n"
        "Controlls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = drive in a circle\n"
        "   B button = drive in a square\n"
        "   X button = drive in a figure eight\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    # Calculate speed from triggers
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    # Calculate angle from left joystick
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    print(f"Speed: {speed:.2f} m/s")
    # TODO (main challenge): Revisit the driving in shapes challenge from lab 1.
    # Using your IMU data, create a more robust version of driving in shapes.

    # When the A button is pressed, add instructions to drive in a circle
    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle...")
        # Implement driving in a circle
        circle_radius = 1.0  # Adjust the radius according to your robot's capabilities
        circle_speed = 0.5  # Adjust the speed for the circle
        circle_angle = 1.0  # Adjust the steering angle for the circle
        rc.drive.set_speed_angle(circle_speed, circle_angle)
        rc.drive.set_circle(circle_radius)

    # When the B button is pressed, add instructions to drive in a square
    if rc.controller.was_pressed(rc.controller.Button.B):
        print("Driving in a square...")
        # Implement driving in a square
        square_speed = 0.5  # Adjust the speed for the square
        square_angle = 1.0  # Adjust the steering angle for the square
        square_side_duration = 2.0  # Adjust the duration for each side of the square
        square_sides = 4  # Adjust the number of sides for the square

        for _ in range(square_sides):
            rc.drive.set_speed_angle(square_speed, square_angle)
            rc.drive.set_heading(90.0)  # Adjust the heading for each side of the square
            rc.drive.run(square_side_duration)

    # When the X button is pressed, add instructions to drive in a figure eight
    if rc.controller.was_pressed(rc.controller.Button.X):
        print("Driving in a figure eight...")
        # Implement driving in a figure eight
        figure_radius = 1.0  # Adjust the radius for the circles in the figure eight
        figure_speed = 0.5  # Adjust the speed for the figure eight
        figure_angle = 1.0  # Adjust the steering angle for the figure eight

        # First circle
        rc.drive.set_speed_angle(figure_speed, figure_angle)
        rc.drive.set_circle(figure_radius)
        rc.drive.run(4.0)  # Adjust the duration for the first circle

        # Second circle
        rc.drive.set_speed_angle(figure_speed, -figure_angle)
        rc.drive.set_circle(-figure_radius)
        rc.drive.run(4.0)  # Adjust the duration for the second circle

    # When the Y button is pressed, add instructions to drive in a shape of your choice
    if rc.controller.was_pressed(rc.controller.Button.Y):
        print("Driving in a custom shape...")
        # Implement driving in a custom shape
        # Add your custom shape movement instructions here

    rc.drive.set_speed_angle(speed, angle)



################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()

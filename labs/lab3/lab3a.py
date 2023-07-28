"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""
import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum
from utils import *
########################################################################################
# Imports
########################################################################################



########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelxerate forward\n"
        "    Right bumper = override safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print the distance at the center of the depth image"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    image = rc.camera.get_color_image()
    cone_distance = 1000
    if image is not None:
    
        cone_mask = get_mask(image, (120, 80, 120), (160,155,200))
        cone_contours = find_contours(cone_mask)
        if len(cone_contours) >0:
            depth_img = rc.camera.get_depth_image()
            largest_contour = get_largest_contour(cone_contours)
            contour_center = get_contour_center(largest_contour)
            try:
                cone_area = cv.contourArea(largest_contour)
                x = remap_range(contour_center[0],0,len(image),len(depth_img),len(depth_img[0]))
                y=remap_range(contour_center[1],0,len(image[1]),len(depth_img),len(depth_img[1]))
                print(x,y)
                cone_distance = depth_img[x][y]
                print(cone_distance)
                return
            except Exception as e:
                print(e)
                pass
    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Calculate the distance of the object directly in front of the car
    center_distance = cone_distance
    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    # Define the desired distance from the cone (in meters)
    if center_distance > 1000:
        center_distance = 1000
    desired_distance = 900  # Approximately 800 cm in meters
    danger = 700
    # Calculate the difference between the desired distance and the actual distance
    speed = remap_range(center_distance,0,1000,-0.7,0.15)



    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # Display the current depth image



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

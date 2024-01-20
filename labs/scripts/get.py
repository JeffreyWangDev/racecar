"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from nptyping import NDArray

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
import rclpy as ros2
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSProfile,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from nptyping import NDArray
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum


rc = racecar_core.create_racecar()


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Print start message
    global current_high_hsv,current_low_hsv,current_hsv_state,current_low_high
   
    current_low_hsv = (0,0,0)
    current_high_hsv = (179,255,255)

    current_hsv_state = 0   # index of hsv
    current_low_high = 0    # 0 for low, 1 for high
    print(
        ">> Script to determine hsv values\n"
        "\n"
        "Controls:\n"
        "    Y button = Change current HSV value up\n"
        "    A button = Change current HSV value down\n"
        "    X button = Toggle what value to change in HSV\n"
        "    B button = Toggle changing upper or lower\n"
    )

current_low_hsv = (0,0,0)           # Start value
current_high_hsv = (179,255,255)    # Start value

current_hsv_state = 0   # index of hsv
current_low_high = 0    # 0 for low, 1 for high

largest_contour = None
image = None
def update():
    global largest_contour
    global image
    image = rc.camera.get_color_image()
    contours =  rc_utils.find_contours(image,current_low_hsv,current_high_hsv)
    largest = rc_utils.get_largest_contour(contours)
    largest_contour = largest
        
    global current_high_hsv,current_low_hsv,current_hsv_state,current_low_high
    if rc.controller.is_down(rc.controller.Button.Y):
        if current_low_high == 0:
            current_low_hsv[current_hsv_state] +=10 # Change for more or less change per press
        else:
            current_high_hsv[current_hsv_state] +=10 # Change for more or less change per press

    if rc.controller.is_down(rc.controller.Button.A):
        if current_low_high == 0:
            current_low_hsv[current_hsv_state] -=10 # Change for more or less change per press
        else:
            current_high_hsv[current_hsv_state]-=10 # Change for more or less change per press

    max = {0:179,1:255,2:255}
    for i in range(3):
        if current_high_hsv[i] > max[i]:
            current_high_hsv[i] = max[i]
        if current_low_hsv[i]<0:
            current_low_hsv[i]=0
        if current_low_hsv[i] > max[i]:
            current_low_hsv[i] = max[i]
        if current_high_hsv[i]<0:
            current_high_hsv[i]=0
    if rc.controller.was_pressed(rc.controller.Button.X):
        current_low_high = (current_low_high+1)%2
    if rc.controller.was_pressed(rc.controller.Button.B):
        if current_hsv_state ==2:
            current_hsv_state =0
        else:
            current_hsv_state+=1

number_to_hsv = {0:"Hue",1:"Saturation",2:"Value"}
def draw_contour(
    image: NDArray,
    contour: NDArray,
    color: Tuple[int, int, int] = (0, 255, 0)
) -> None:
    """
    Draws a contour on the provided image.

    Args:
        image: The image on which to draw the contour.
        contour: The contour to draw on the image.
        color: The color to draw the contour in BGR format.
    """
    if contour is not None:
        cv.drawContours(image, [contour], 0, color, 3)
    return image
def slow():
    if image is not None and largest_contour is not None:
        image_a = draw_contour(image, largest_contour)
        rc.display.show_color_image(image_a)
    else:
        print("Image not found/Contour not found")
    print("-"*10)
    print("Current HSV values: ", str(current_low_hsv), str(current_high_hsv))
    print("Current HSV mode: ",number_to_hsv[current_hsv_state])
    print("Current HSV high low: ","Low" if current_low_high ==0 else "High")
    print("-"*10)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
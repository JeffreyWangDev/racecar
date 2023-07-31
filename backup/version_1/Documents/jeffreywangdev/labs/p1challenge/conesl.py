import sys
import cv2 as cv
import numpy as np
from enum import Enum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
class Mode(Enum):
    searching = 0
    orange = 1
    purple = 2
    linear = 3

class Color(Enum):
    orange = 0
    purple = 1
    BOTH = 2

curr_mode = Mode.searching
color_priority = Color.orange

speed = 0.0
angle = 0.0
last_distance = 0

counter = 0

rc = racecar_core.create_racecar()

orange = ((130, 106, 128), (150, 170, 190)) 
purple = ((1, 226, 229), (25, 255, 255))

########################################################################################
# Utility Functions
########################################################################################
global contour_R
global contour_B
def update_contours(color_image):
    """
    Finds contoours for the purple and orange cone using color image
    """

    MIN_CONTOUR_AREA = 500

    # If no image is fetched
    if color_image is None:
        contour_center = None
        contour_area = 0
        print("no colors dawg")

    # If an image is fetched successfully
    else:
        # Find all of the orange contours
        contours_R = rc_utils.find_contours(color_image, orange[0], orange[1])
        print("see orange")

        # Find all of the purple contours
        contours_B = rc_utils.find_contours(color_image, purple[0], purple[1])
        print("see purp")

        # Select the largest contour from orange and purple contours
        contour_R = rc_utils.get_largest_contour(contours_R, MIN_CONTOUR_AREA)
        contour_B = rc_utils.get_largest_contour(contours_B, MIN_CONTOUR_AREA)
        #print(rc_utils.get_contour_area(contour_B))

        # Draw a dot at the center of this contour in orange
        if contour_R is not None and contour_B is not None: # checks if both are valid

            contour_area_R = rc_utils.get_contour_area(contour_R)
            contour_area_B = rc_utils.get_contour_area(contour_B)
            # if the contour areas are similar enough, indicate that it is a checkpoint
            # If orange contour is bigger than the purple one
        elif contour_area_R > contour_area_B:
            return contour_R, Color.orange

            # If purple contour is equal to or bigger than the orange one
            else:
                return contour_B, Color.purple

        elif contour_R is None and contour_B is not None:
            return contour_B, Color.purple

        elif contour_B is None and contour_R is not None: 
            return contour_R, Color.orange

        else:
            # No contours found
            return None, None

########################################################################################
# Environment Interaction Functions
########################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """

    # Globals
    global curr_mode

    # Have the car begin at a stop
    rc.drive.stop()
    curr_mode = Mode.searching
    rc.drive.set_max_speed(0.75)

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

def update():
    pass
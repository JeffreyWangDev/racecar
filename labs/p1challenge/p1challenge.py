"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
"""

########################################################################################
# Imports
########################################################################################

import sys
from typing import Tuple
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from utils import *
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
    print(">> Phase 1 Challenge: Cone Slaloming")

def detect_cone(color_image: np.ndarray) -> Tuple[bool, int]:
    image = rc.camera.get_color_image()
    if image is not None:
        cone_mask = get_mask(image, (120, 80, 120), (160,155,200))
        cone_contours = find_contours(cone_mask)
        if len(cone_contours) >0:
            depth_img = rc.camera.get_depth_image()
            largest_contour = get_largest_contour(cone_contours)
            contour_center = get_contour_center(largest_contour)
            try:
                x = remap_range(contour_center[0],0,len(image),len(depth_img),len(depth_img[0]))
                y = remap_range(contour_center[1],0,len(image[1]),len(depth_img),len(depth_img[1]))
                cone_distance = get_average(depth_img[x-15:x+15,y-15:y+15])
                print(cone_distance)
                return 1,cone_distance
            except Exception as e:
                print(e)
                return 0
        cone_mask = get_mask(image, (120, 80, 120), (160,155,200))
        cone_contours = find_contours(cone_mask)
        if len(cone_contours) >0:
            depth_img = rc.camera.get_depth_image()
            largest_contour = get_largest_contour(cone_contours)
            contour_center = get_contour_center(largest_contour)
            try:
                x = remap_range(contour_center[0],0,len(image),len(depth_img),len(depth_img[0]))
                y = remap_range(contour_center[1],0,len(image[1]),len(depth_img),len(depth_img[1]))
                cone_distance = get_average(depth_img[x-15:x+15,y-15:y+15])
                print(cone_distance)
                return 2,cone_distance
            except Exception as e:
                print(e)
                return 0
        
            
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Slalom between red and blue cones.  The car should pass to the right of
    # each red cone and the left of each blue cone.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

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
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import Enum
from utils import *
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
class Mode(Enum):
    searching = 0
    Purple = 2
    Orange = 1
    Stop = rc.drive.stop()

class Color(Enum):
    Purple = 1
    Orange = 0
    Red = 2

curr_mode = Mode.searching
color_priority = Color.Orange

speed = 0.0
angle = 0.0
last_distance = 0

counter = 0


Purple = ((120, 120, 122), (179, 255, 255)) 
Orange = ((166, 254-20, 254-10), (24, 255, 255))
########################################################################################
# Functions
########################################################################################
def update_contours(color_image):

    MIN_CONTOUR_AREA = 40
    if color_image is None:
        contour_center = None
        contour_area = 0

    else:
        contours_O = rc_utils.find_contours(color_image, Orange[0], Orange[1])

        contours_P = rc_utils.find_contours(color_image, Purple[0], Purple[1])
        contour_O = rc_utils.get_largest_contour(contours_O, MIN_CONTOUR_AREA)
        contour_P = rc_utils.get_largest_contour(contours_P, MIN_CONTOUR_AREA)
        



        if contour_O is not None and contour_P is not None: 

            contour_area_O = rc_utils.get_contour_area(contour_O)
            contour_area_P = rc_utils.get_contour_area(contour_P)

            if abs(contour_area_O - contour_area_P) < 700:
                print(abs(contour_area_O - contour_area_P))
                return None, Color.Purple

            elif contour_area_O > contour_area_P:
                return contour_O, Color.Orange

            else:
                return contour_P, Color.Purple

        elif contour_O is None and contour_P is not None:
            return contour_P, Color.Purple

        elif contour_P is None and contour_O is not None: 
            return contour_O, Color.Orange

        else:
            return None, None

def start():
    global curr_mode

    rc.drive.stop()
    curr_mode = Mode.searching

    print(">> Phase 1 Challenge: Cone Slaloming")

def update_slow():
	pass

def update():
    global curr_mode
    global speed
    global angle
    global color_priority
    global last_distance
    global counter

    speed = 0.0
    angle = 0.0
    distance = 5000
    speed_multiplier = 1
    distance_param = 200
    depth_image = rc.camera.get_depth_image()
    color_image = rc.camera.get_color_image()

    camera_height = (rc.camera.get_height() // 10) * 10
    camera_width = (rc.camera.get_width() // 10) * 10

    top_left_inclusive = (0, rc.camera.get_width() - camera_width)
    bottom_right_exclusive = ((camera_height, camera_width))

    rc_utils.crop(color_image, top_left_inclusive, bottom_right_exclusive)
    rc_utils.crop(depth_image, top_left_inclusive, bottom_right_exclusive)

    contour, color = update_contours(color_image)

    color_image_display = np.copy(color_image)
    if contour is not None:
        contour_center = rc_utils.get_contour_center(contour)
        rc_utils.draw_contour(color_image_display, contour)
        rc_utils.draw_circle(color_image_display, contour_center)
        x = remap_range(contour_center[0],0,len(color_image),0,len(depth_image[0]))
        y = remap_range(contour_center[1],0,len(color_image[1]),0,len(depth_image[1]))
        try:
            distance = get_average(depth_image[x-15:x+15,y-15,y+15])
        except:
            distance = depth_image[x,y]
        last_distance = distance
        print(f"Distance: {distance}")

    else:
        curr_mode = Mode.searching

    if color == Color.Orange:
        curr_mode = Mode.Orange
        color_priority = Color.Purple
    elif color == Color.Purple:
        curr_mode = Mode.Purple
        color_priority = Color.Purple
    elif color == Color.Red:
        curr_mode = Mode.Stop
    else:
        curr_mode = Mode.searching


    if curr_mode == Mode.Orange and (distance < distance_param):
        angle = rc_utils.remap_range(contour_center[1], 0, camera_width, 0.3, 1)
        angle *= rc_utils.remap_range(last_distance, 200, 50, 0, 2)
        print("Orange, ANGLE:", angle)
        counter = 0
    elif curr_mode == Mode.Purple and (distance < distance_param):
        angle = rc_utils.remap_range(contour_center[1], 0, camera_width, -1, -0.3)
        angle *= rc_utils.remap_range(last_distance, 50, 200, 2, 0)
        print("Purple, ANGLE:", angle)
        counter = 0
    elif (curr_mode == Mode.Purple or curr_mode == Mode.Purple) and distance >= distance_param:
        if curr_mode == Mode.Purple:
            angle = rc_utils.remap_range(contour_center[1], 0, camera_width, -1, 1)
        elif curr_mode == Mode.Purple:
            angle = rc_utils.remap_range(contour_center[1], 0, camera_width, -1, 1)
        print("waiting")
        counter = 0
    elif curr_mode == Mode.Stop:
        print("got here")
        angle = 0
        counter = 0
    else:
        if color_priority == Color.Orange:
            angle = rc_utils.remap_range(last_distance, 0, 100, -0.3, -0.65) # drive left to return
        else:
            angle = rc_utils.remap_range(last_distance, 0, 100, 0.3, 0.65) # drive right to return


    ###########
    # TEMP MANUAL CONTROLS
    ###########
    #speed -= rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    #speed += rc.controller.get_trigger(rc.controller.Trigger.RIGHT)


    # Clamping functions
    angle = rc_utils.clamp(angle, -1, 1)
    speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.05)
    speed = rc_utils.remap_range(last_distance, 60, 150, 0.1, 0.98)
    speed *= speed_multiplier
    speed = rc_utils.clamp(speed, -1, 1)


    # Displaying the color camera that was drawn on
    rc.display.show_color_image(color_image_display)

    # Setting the speed and angle of the car
    rc.drive.set_speed_angle(speed, angle)
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

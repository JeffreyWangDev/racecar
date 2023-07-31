"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2A - Color Image Line Following
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
# ########################################################################################
# import rclpy as ros2
# from rclpy.qos import (
#     QoSDurabilityPolicy,
#     QoSHistoryPolicy,
#     QoSReliabilityPolicy,
#     QoSProfile,
# )
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
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
from utils import *


rc = racecar_core.create_racecar()

speed = 0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
image = None
speed_current_val = 0.0
speed_accumulated_error = 0.0
speed_last_error = 0.0
dt = 1.0
speed_speed = 0.2
speed_PID_P = 0.4
speed_PID_I = 0.1
speed_PID_D = 0.08
speed_V0 = 0
speed_V1 = 0
speed_set_speed = 0
speed_dv = 0
speed_a_list=[0,0,0,0,0]
speed_average_s = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
cone_area = 0
largest_contour = None
cone_distance = 1000
def average(lista):
    average_v = 0
    for i in lista:
        average_v+=i
    return average_v/len(lista)
def update_contour(cone = False):
    global DRIVE
    global image
    global cone_area
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global angle
    global red
    global current_state
    global largest_contour
    global cone_distance
    largest_contour = None

    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)

        # Crop the image to the floor directly in front of the car
        colors = {
            "green":[],
            "red":[],
            "yellow": [],
            "blue":[]


        }
        hsv = [
            colors[""]
        ]

        image_copy = np.copy(image)
        cone_mask = get_mask(image_copy, hsv[2][0], hsv[2][0])
        cone_contours = find_contours(cone_mask)
        if len(cone_contours) >0:
            largest_contour = get_largest_contour(cone_contours)
            contour_center = get_contour_center(largest_contour)
            try:
                current_state = states.parking_cone
                return
            except Exception as e:
                print(e)
                pass
        if not cone:
            image = image[100:len(image)]
            first_mask = get_mask(image, hsv[0][0], hsv[0][1])
            first_contours = find_contours(first_mask)
            if len(first_contours) > 0:
                largest_contour = get_largest_contour(first_contours)
                contour_center = get_contour_center(largest_contour)
                try:
                    contour_area = cv.contourArea(largest_contour)
                    return
                except:
                    pass

            second_mask = get_mask(image, hsv[1][0], hsv[1][1])
            second_contours = find_contours(second_mask)
            if len(second_contours) > 0:
                largest_contour = get_largest_contour(second_contours)
                contour_center = get_contour_center(largest_contour)
                try:
                    contour_area = cv.contourArea(largest_contour)
                    return
                except:
                    pass
            else:
                contour_center = None
                contour_area = 0

target_val = 160
current_val = 0.0
accumulated_error = 0.0
last_error = 0.0
dt = 1.0

current_state = states.following_line
red=True
def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global red
    global speed_speed,speed_current_val,speed_accumulated_error,speed_last_error
    global target_val,angle,target_val,current_val,accumulated_error,last_error
    red=True
    # Initialize variables
    speed = 0.16
    angle = 0
    target_val = 160
    current_val = 0.0
    accumulated_error = 0.0
    last_error = 0.0
    speed_current_val = 0.0
    speed_accumulated_error = 0.0
    speed_last_error = 0.0
    dt = 1.0
    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)
    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)
    
    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    A button = print current speed and angle\n"
        "    B button = print contour center and area"
    )
PID_P = 0.225 #0.8*0.6
PID_I = 0 #0.6
PID_D = 0.1 #0.17
def update():
    global speed_current_val,speed_accumulated_error,speed_last_error,dt,speed_V0,speed_V1,speed_set_speed,speed_speed,speed_PID_P,speed_dv,speed_a_list,speed_average_s
    global speed
    global red
    if rc.controller.was_pressed(rc.controller.Button.B):
        speed_speed-=0.1
    if rc.controller.was_pressed(rc.controller.Button.A):
        speed_speed+=0.1
    global PID_P,PID_I,PID_D
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global angle
    global current_val,accumulated_error,last_error,dt,current_state
    dt=rc.get_delta_time()
    update_contour()
    
    dt=rc.get_delta_time()
    speed_dv = rc.physics.get_angular_velocity()
    speed_V1 =speed_dv[2]/dt
    speed_v = speed_V1-speed_V0
    speed_a_list.append(speed_v)
    speed_a_list.pop(0)
    speed_average_v=average(speed_a_list)
    speed_V0 = speed_dv[2]/dt
    speed_set_speed,speed_accumulated_error,speed_last_error = pid_control(speed_PID_P, speed_PID_I, speed_PID_D, speed_speed, speed_average_v, speed_accumulated_error, speed_last_error, dt)
    speed_set_speeda = clamp(speed_set_speed, -1, 1)
    speed_average_s.append(speed_set_speeda)
    speed_average_s.pop(0)
    if current_state==states.following_line:
        if contour_center is not None:

            anglea,accumulated_error,last_error = pid_control(PID_P, PID_I, PID_D, 160, contour_center[1], accumulated_error, last_error, rc.get_delta_time())
            angle = remap_range(anglea, -320,320, 1, -1)
            angle= clamp(angle, -1, 1)
        rc.drive.set_speed_angle(average(speed_average_s), angle)

    if current_state == states.parking_cone:
        print("Cone")

    



        
    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle, "PID: ",PID_P,PID_I,PID_D)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    image_a = draw_contour(image, largest_contour)
    rc.display.show_color_image(image_a)
    print(speed_speed,speed_set_speed,average(speed_a_list))
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
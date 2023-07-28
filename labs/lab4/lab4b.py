"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
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
    print(">> Lab 4B - LIDAR Wall Following")

def get_average(list):
    return abs(sum(list)/len(list)-list.count(0))

def update():
    angle = 0 
    scan = rc.lidar.get_samples()
    right_dist=scan[160:200]
    left_dist = scan[520:560]
    front_right_dist = scan[75:115]
    front_left_dist = scan[605:645]
    font_dist = scan[0:10]+scan[710:720]
    con = [0.25,0.5,0.25]
    r_avg = get_average(list(right_dist))
    l_avg = get_average(list(left_dist))
    right_dist = list(right_dist)
    left_dist = list(left_dist)
    
    print(r_avg,l_avg)
    for i in range(len(right_dist)):
        if right_dist[i] > 3*r_avg or right_dist[i] < r_avg/3:
            right_dist[i] = r_avg
    for i in range(len(left_dist)):
        if left_dist[i] > 3*l_avg or left_dist[i] < l_avg/3:
            left_dist[i] = l_avg
    
    left_dist = np.array(left_dist)
    right_dist = np.array(right_dist)
    
    new_right = convolution(left_dist,con)
    new_left = convolution(right_dist,con)
    new_front_right = convolution(front_right_dist,con)
    new_front_left = convolution(front_left_dist,con)
    new_font = convolution(font_dist,con)
    r_dist_avg = abs(np.mean(new_right))
    l_dist_avg = abs(np.mean(new_left))
    r_front_dist_avg = abs(np.mean(new_front_right))
    l_front_dist_avg = abs(np.mean(new_front_left))
    
    front_avg_dist = abs(np.mean(new_font))
    
    print(front_avg_dist,r_front_dist_avg-l_front_dist_avg,r_dist_avg-l_dist_avg)
    if front_avg_dist < 700:
        angle = remap_range(r_front_dist_avg-l_front_dist_avg,-40,40,-1,1)
        print("first")
    else:
        angle += remap_range(r_dist_avg-l_dist_avg,-40,40,-1,1)

        print("second")
    #print("second")

    angle = clamp(angle,-1,1)
    print(angle)
    rc.drive.set_speed_angle(1,angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()


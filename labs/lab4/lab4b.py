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
import utils
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
windows = np.array(())
start_degrees = 90
total_degrees = 180
total_windows = 10
max_speed = .2

p = 0
i = 0
d = 0

a_error = 0
l_error = 0
dt=1
current = 0

########################################################################################
# Functions
########################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()
    global max_speed
    rc.drive.set_max_speed(max_speed)

    # Create scanning windows
    global windows
    global total_degrees
    global total_windows
    global start_degrees
    window_size = round(total_degrees / total_windows)
    for i in range(total_windows):
        windows = np.append(windows, i * window_size)
        windows = np.append(windows, (i+1) * window_size - 1)

    windows = windows + start_degrees
    windows = windows.reshape(-1, 2)
    print(windows)

    global a_error,l_error,dt,current
        
    a_error = 0
    l_error = 0
    dt=1
    current = 0

    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")
    

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Follow the wall to the right of the car without hitting anything.

    global windows # getting all the angle windows that the car will look at
    global total_degrees
    global total_windows
    global start_degrees
    speed = 0.15

    # First grab lidar data
    scan = rc.lidar.get_samples()

    # Get the (!!!average) closest distance for each window using lidar scan data
    windows_distances = np.array(())
    for window in windows:
        _, window_distance = rc_utils.get_lidar_closest_point(scan, window)
        windows_distances = np.append(windows_distances, window_distance)
    
    windows_distances = windows_distances.reshape(-1, 1)

    # Turns to the angle
    angle_index = np.argmax(windows_distances)
    angle_degrees = np.mean(windows[angle_index])
    angle = rc_utils.remap_range(angle_degrees, start_degrees, start_degrees + total_degrees - 1, -1, 1) * 2
    angle = rc_utils.clamp(angle, -1, 1)

    # Manual speed control
    """
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    """

    # If the distance in front of you is very close, SLOW DOWN
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, (-2, 2))
    if forward_dist < 250 * max_speed and abs(speed) == speed:
        multiplier = rc_utils.remap_range(forward_dist, 200 * max_speed, 300 * max_speed, 0 * max_speed, 0.7 * max_speed)
        speed = multiplier * speed

    print(f"angle degrees: {angle_degrees}, angle {angle}")
    rc.drive.set_speed_angle(speed, angle)



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
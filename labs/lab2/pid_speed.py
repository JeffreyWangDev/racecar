"""
Copyright Jeffrey Wang
MIT License
Summer 2023

"""

########################################################################################
# Imports
if True:
    import sys
    import cv2 as cv
    import numpy as np
    from nptyping import NDArray

    sys.path.insert(1, "../../library")
    import racecar_core
    import racecar_utils as rc_utils
    from utils import *
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
########################################################################################


rc = racecar_core.create_racecar()

target_val = 0.5
current_val = 0.0
accumulated_error = 0.0
last_error = 0.0
dt = 1.0
speed = 0 
def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global red
    red=True
    # Initialize variables
    current_val = 0.0
    accumulated_error = 0.0
    last_error = 0.0
    dt = 1.0
    # Set initial driving speed and angle
    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)
    
    # Print start message
    print(
        ">> Lab 2 - PID speed control\n"
        "\n"
        "Controls:\n"
        "    A button = speed up\n"
        "    B button = speed down"
    )
PID_P = 5
PID_I = 0
PID_D = 0
V0 = 0
def update():
    global speed
    if rc.controller.is_down(rc.controller.Button.B):
        speed-=0.01
    if rc.controller.was_pressed(rc.controller.Button.A):
        speed+=0.01
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    global current_val,accumulated_error,last_error,dt,V0
    dt=rc.get_delta_time()
    accel = rc.physics.get_linear_acceleration()
    va = get_v(V0,accel[2],dt)
    set_speed,accumulated_error,last_error = pid_control(PID_P, PID_I, PID_D, speed, va, accumulated_error, last_error, dt)
    # angle = remap_range(anglea, -320,320, 1, -1)
    print(set_speed)
    set_speed= clamp(set_speed, -1, 1)
    rc.drive.set_speed_angle(set_speed, angle)

    



        
    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle, "PID: ",PID_P,PID_I,PID_D)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    print(speed)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()


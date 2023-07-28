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
queue= []
speed_a_list = [0,0,0,0,0,0,0,0,0,0,0]
speed_V0  = 0
def start():
    """
    This function is run once every time the start button is pressed
    """
    global queue,speed_a_list,speed_V0
    queue = []
    speed_a_list = [0,0,0,0,0,0,0,0,0,0,0]
    speed_V0  = 0
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
def update():
    global queue,speed_V0,speed_a_list
    dt=rc.get_delta_time()
    speed_dv = rc.physics.get_angular_velocity()
    speed_V1 =speed_dv[2]/dt
    speed_v = speed_V1-speed_V0
    speed_a_list.append(speed_v)
    speed_a_list.pop(0)
    if queue:
        if queue[0][0]<=0:
            queue.pop(0)
            rc.drive.stop()
        else:
            queue[0][0]-=get_distance(speed_v,dt)
            rc.drive.set_speed_angle(0.2,queue[0][1])
    if rc.controller.was_pressed(rc.controller.Button.A):
        queue.append([0.5,0])
    
def get_distance(v,dt):
    return v*dt

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


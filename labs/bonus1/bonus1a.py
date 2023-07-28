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
speed = 0.2
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

V0 = 0
V1 = 0
av = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

def get_average(to):
    return sum(to)/len(to)

def update():
    global av

    angle,speed = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0],rc.controller.get_joystick(rc.controller.Joystick.LEFT)[1]
    global current_val,accumulated_error,last_error,dt,V0,V1
    dt=rc.get_delta_time()
    dv = rc.physics.get_angular_velocity()
    dva = rc.physics.get_linear_acceleration()
    print(dv,dva)
    V1 =dv[2]/dt
    v = V1-V0
    V0 = dv[2]/dt
    av.pop(0)
    av.append(v)
    # print(av)
    # print(get_average(av))
    rc.drive.set_speed_angle(speed, angle)
        



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()


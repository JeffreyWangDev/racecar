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

speed_current_val = 0.0
speed_accumulated_error = 0.0
speed_last_error = 0.0
dt = 1.0
speed_speed = 0.2
speed_PID_P = 0.3
speed_PID_I = 0.09
speed_PID_D = 0.06
speed_V0 = 0
speed_V1 = 0
speed_set_speed = 0
speed_dv = 0
speed_a_list=[0,0,0,0,0]
speed_average_s = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
def average(lista):
    average_v = 0
    for i in lista:
        average_v+=i
    return average_v/len(lista)
def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed_speed,speed_current_val,speed_accumulated_error,speed_last_error
    
    # Initialize variables
    speed_current_val = 0.0
    speed_accumulated_error = 0.0
    speed_last_error = 0.0
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


def update():
    if rc.controller.was_pressed(rc.controller.Button.B):
        speed_speed-=0.1
    if rc.controller.was_pressed(rc.controller.Button.A):
        speed_speed+=0.1
    if rc.controller.was_pressed(rc.controller.Button.Y):
        speed_PID_P+=0.01
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    global speed_current_val,speed_accumulated_error,speed_last_error,dt,speed_V0,speed_V1,speed_set_speed,speed_speed,speed_PID_P,speed_dv,speed_a_list,speed_average_s
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
    rc.drive.set_speed_angle(average(speed_average_s), angle)
        
    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.X):
        print("Speed:", speed, "Angle:", angle, "PID: ",speed_PID_P,speed_PID_I,speed_PID_D)
        print(a_list)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    print(speed_speed,speed_set_speed,average(speed_a_list))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()


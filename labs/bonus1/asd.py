"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Final code for 7/28/2023 line + cone + wall + parking

"""

########################################################################################
# Imports start
########################################################################################

import sys
from nptyping import NDArray
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
from typing import Any, Tuple, List, Optional
from enum import Enum
from utils import *
from abc import ABC, abstractmethod

########################################################################################
# Imports end
########################################################################################

rc = racecar_core.create_racecar()

class Color(ABC):
    @abstractmethod
    def __init__(self,lower:tuple,upper:tuple) -> None:
        self.lower = lower
        self.upper = upper
    @abstractmethod
    def lower(self):
        return self.lower
    @abstractmethod
    def upper(self):
        return self.upper
class Colors:
    class Cones:
        red = [(),()]
        blue = [(),()]
    class Lines:
        red = [(),()]

class Pid:
    class Speed:
        def __init__(self) -> None:
            self.speed_speed = 0.2                      # Set speed
            self.speed_PID_P = 0.4                      # PID P term
            self.speed_PID_I = 0.1                      # PID I term
            self.speed_PID_D = 0.08                     # PID D term
            self.speed_V0 = 0                           # Previous speed
            self.speed_V1 = 0                           # Current speed
            self.speed_power = 0                        # Current power level
            self.speed_velocity_list=[0,0,0,0,0]        # List of speed changes
            self.speed_power_list=[0 for i in range(10)]# List of average speed changes
            self.speed_current_val = 0.0                # Current speed
            self.speed_accumulated_error = 0.0          # PID addtive error for I
            self.speed_last_error = 0.0                 # Previous PID error
        def update(self,dt:float,angular_v:tuple) -> float:
            """
            Updates the speed PID control
            
            Args:
                dt (float): Delta time
                angular_v (tuple): Angular velocity
            Returns:
                float: Power level for the motor
            """
            self.speed_V1 = angular_v[2]/dt                                                             # Change in speed
            self.speed_v = self.speed_V1-self.speed_V0                                                  # Current speed
            self.speed_velocity_list.append(self.speed_v)                                               # Add current speed to list
            self.speed_velocity_list.pop(0)                                                             # Remove oldest speed from list
            self.speed_average_v=average(self.speed_velocity_list)                                      # Average speed
            self.speed_power,self.speed_accumulated_error,self.speed_last_error = pid_control(          # PID control
                self.speed_PID_P, self.speed_PID_I, self.speed_PID_D,                                   # PID constants
                self.speed_speed, self.speed_average_v,                                                 # Set speed, current speed
                self.speed_accumulated_error,                                                                # PID addtive error for I
                self.speed_last_error, dt                                                                    # Previous PID error, delta time
                )                                                                   
            self.speed_powera = clamp(self.speed_power, -1, 1)                                          # Clamp power
            self.speed_power_list.append(self.speed_powera)                                             # Add current power to list
            self.speed_power_list.pop(0)                                                                # Remove oldest power from list
            self.speed_V0 = self.speed_V1                                                               # Set previous speed to current speed
            return average(self.speed_power_list)                                                       # Return average power
    class Line:
        pass
        
class Camera:
    class Line:
        def __init__(self) -> None:
            self.contour_center = None
            self.contour_area = 0
            self.largest_contour
        def get_color(image,color:Colors.Lines=None) -> float:
            image_copy = image[100:len(image)]
            if color:
                return rc_utils.get_largest_contour(rc_utils.find_contours(image_copy, Colors.Lines.red, color[1]))
            return {
                "yellow": rc_utils.find_contours(image, YELLOW[0], YELLOW[1]),
                "green": rc_utils.find_contours(image, GREEN[0], GREEN[1]),
                "orange": rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
            }
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
from abc import ABC, abstractmethod
from utils import *
########################################################################################
# Imports end
########################################################################################
def average(lista):
    return sum(lista)/len(lista)
rc = racecar_core.create_racecar()

class Colors:
    class Cones:
        class Purple:
            lower_value = (142,86,54)
            upper_value = (162,156,135)
            name = "Purple"
    class Lines:
        class Red: 
            lower_value = (135,169,145)
            upper_value = (179,255,255)
            name="Red"
        class Blue:
            lower_value = (64,66,164)
            upper_value = (124,126,224)
            name="Blue"
        class Green:
            lower_value = (30,49,125)
            upper_value = (80,169,245)
            name="Green"
        class Yellow:
            lower_value = (1,51,195)
            upper_value = (46,171,255)
            name = "Yellow"

class Pid:
    def __init__(self) -> None:
        self.speed = self.Speed()
        self.line = self.Line()
    class Speed:
        def __init__(self) -> None:
            self.speed_speed = 0.2                        # Set speed
            self.__speed_PID_P = 0.4                      # PID P term
            self.__speed_PID_I = 0.1                      # PID I term
            self.__speed_PID_D = 0.08                     # PID D term
            self.__speed_V0 = 0                           # Previous speed
            self.__speed_V1 = 0                           # Current speed
            self.__speed_power = 0                        # Current power level
            self.__speed_velocity_list=[0,0,0,0,0]        # List of speed changes
            self.__speed_power_list=[0 for i in range(10)]# List of average speed changes
            self.__speed_accumulated_error = 0.0          # PID addtive error for I
            self.__speed_last_error = 0.0                 # Previous PID error
        def update(self,dt:float,angular_v:tuple) -> float: 
            """
            Updates the speed PID control
            
            Args:
                dt (float): Delta time
                angular_v (tuple): Angular velocity
            Returns:
                float: Power level for the motor
            """
            self.__speed_V1 = angular_v[2]/dt                                                               # Change in speed
            self.__speed_v = self.__speed_V1-self.__speed_V0                                                # Current speed
            self.__speed_velocity_list.append(self.__speed_v)                                               # Add current speed to list
            self.__speed_velocity_list.pop(0)                                                               # Remove oldest speed from list
            self.__speed_average_v=average(self.__speed_velocity_list)                                      # Average speed
            self.__speed_power,self.__speed_accumulated_error,self.__speed_last_error = pid_control(        # PID control
                self.__speed_PID_P, self.__speed_PID_I, self.__speed_PID_D,                                 # PID constants
                self.speed_speed, self.__speed_average_v,                                                   # Set speed, current speed
                self.__speed_accumulated_error,                                                             # PID addtive error for I
                self.__speed_last_error, dt                                                                 # Previous PID error, delta time
                )                                                                   
            speed_power = clamp(self.__speed_power, -1, 1)                                                  # Clamp power
            self.__speed_power_list.append(speed_power)                                                     # Add current power to list
            self.__speed_power_list.pop(0)                                                                  # Remove oldest power from list
            self.speed_V0 = self.__speed_V1                                                                 # Set previous speed to current speed
            return average(self.__speed_power_list)                                                         # Return average power
    class Line:
        def __init__(self) -> None:
            self.__line_PID_P = 0.4                      # PID P term
            self.__line_PID_I = 0.1                      # PID I term
            self.__line_PID_D = 0.08                     # PID D term
            self.__line_accumulated_error = 0.0          # PID addtive error for I
            self.__line_last_error = 0.0                 # Previous PID error
            self.__line_set_point = 160
        def update(self,dt:float,x_pos:int) -> float:
            angle,self.__line_accumulated_error,self.__line_last_error = pid_control(self.__line_PID_P, self.__line_PID_I, self.__line_PID_D, self.__line_set_point, x_pos, self.__line_accumulated_error, self.__line_last_error, dt)
            angle = remap_range(angle, -320,320, 1, -1)
            angle= clamp(angle, -1, 1)
            return angle
        
class Camera:
    def __init__(self) -> None:
        self.aruco = self.Aruco()
        self.line = self.Line()
    class Aruco:
        def __init__(self) -> None:
            self.marker = None
            self.id = -1
            self.center = (0,0)
        def update(self,image) -> int:
            maker = rc_utils.get_ar_markers(image)
            if maker is not None:
                self.marker = maker[0]
                self.id = self.marker.get_id()
                corners = self.marker.get_corners()
                self.center = ((corners[0][0]-corners[-1][0])/2,(corners[0][1]-corners[-1][1])/2)
                return self.id
            return None
    
    class Line:
        def __init__(self) -> None:
            self.contour_center = 160
            self.color = None

        @staticmethod
        def find_contours(image,color=None) -> float:
            
            if color!=None:
                print(color.name)
                return rc_utils.get_largest_contour(rc_utils.find_contours(image, color.lower_value, color.upper_value))
            return {
                "blue": rc_utils.find_contours(image, Colors.Lines.Blue.lower_value, Colors.Lines.Blue.upper_value),
                "green": rc_utils.find_contours(image, Colors.Lines.Green.lower_value, Colors.Lines.Green.upper_value),
                "red": rc_utils.find_contours(image, Colors.Lines.Red.lower_value, Colors.Lines.Red.upper_value),
                "yellow": rc_utils.find_contours(image, Colors.Lines.Yellow.lower_value, Colors.Lines.Yellow.upper_value),
            }
        @staticmethod
        def find_largest_contour(contour) -> Any:            
            return rc_utils.get_largest_contour(contour)
        @staticmethod
        def find_center_contour(contour) -> tuple:
            return rc_utils.get_contour_center(contour) 
        @staticmethod
        def preprocess_image(image) -> Any:
            image_copy = image[100:len(image)]
            return image_copy
        def update(self,image) -> int:
            color_priority = [Colors.Lines.Blue,Colors.Lines.Green,Colors.Lines.Red,Colors.Lines.Yellow]
            for i in color_priority:
                contour = self.find_contours(image,i)
                if contour is not None:
                    if len(contour) > 1:
                        largest = self.find_largest_contour(contour)
                        if largest is not None:
                            center = self.find_center_contour(largest)
                            self.contour_center = center[0]
                            self.color = i.name
                            return center[0]

class Car:
    def __init__(self) -> None:
        self.pid=Pid()
        self.camera=Camera()
        
car = Car()
def start():
   global car
   rc.set_update_slow_time(0.5)

def update():
    global car
    angle = 0
    dt = rc.get_delta_time()
    x_pos = car.camera.line.update(rc.camera.get_color_image())
    if x_pos:
        angle = car.pid.line.update(dt,x_pos) 
    speed = 0.14    
    rc.drive.set_speed_angle(speed,angle)

def slow_update():
    print("Contour center: ",car.camera.line.contour_center)
    print("Color detected: ",car.camera.line.color)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update,slow_update)
    rc.go()

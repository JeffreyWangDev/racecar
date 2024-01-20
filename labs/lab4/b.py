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
from enum import IntEnum
import math
########################################################################################
# Imports end
########################################################################################
def average(lista):
    return sum(lista)/len(lista)
rc = racecar_core.create_racecar()

class Colors:
    class Cones:
        class Blue:
            lower_value = (0,0,0)
            upper_value = (0,0,0)
    class Lines:
        class Red: 
            lower_value = (116,129,192)
            upper_value = (179,248,255)
        class Blue:
            lower_value = (64,66,164)
            upper_value = (124,126,224)
        class Green:
            lower_value = (30,49,125)
            upper_value = (80,169,245)
        class Yellow:
            lower_value = (1,51,195)
            upper_value = (46,171,255)

class Pid:
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
            self.__speed_average_v=average(self.__speed_velocity_list)                                  # Average speed
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
            return average(self.__speed_power_list)                                                     # Return average power
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

        @staticmethod
        def find_contours(image,color=None) -> float:
            if color:
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
                            return center[0]

class Car:
    def __init__(self) -> None:
        self.pid_speed = Pid.Speed()
        self.pid_line = Pid.Line()
        self.camera_line = Camera.Line()
        self.camera_aruco = Camera.Aruco()

car = Car()

def start():
   global car
   

def update():
    global car
    angle = 0
    dt = rc.get_delta_time()
    x_pos = car.camera_line.update(rc.camera.get_color_image())
    if x_pos:
        angle = car.pid_line.update(dt,x_pos)
    
    speed = car.pid_speed.update(dt,rc.physics.get_angular_velocity())
    rc.drive.set_speed_angle(speed,angle)
    
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()



########################################################################################
# Imports
########################################################################################

###########################################################

    rc = racecar_core.create_racecar()

    MIN_CONTOUR_AREA = 1200

    global queue
    queue=[]

    coneColor = None

    ORANGE =((8, 100, 100), (20, 255, 255))
    PURPLE = ((127, 83, 120), (160,255, 255))

    contour_center = None  # The (pixel row, pixel column) of contour
    contour_area = 0  # The area of contour

    CROP_FLOOR = ((100,0), (rc.camera.get_height(), rc.camera.get_width()))


    # Add any global variables here
    ########################################################################################
    # States
    ########################################################################################
    class State(IntEnum):
        Search = 0
        orangeCurve = 1
        purpleCurve = 2
    cur_state: State = State.Search
    ########################################################################################
    # Functions
    ########################################################################################

class ConeSlalom
    def start():
        """
        This function is run once every time the start button is pressed
        """
        global speed
        global angle
        global cur_state
        global queue

        # Initialize variables
        speed = 0
        angle = 0

        cur_state = State.Search

        queue.clear()
        # Set initial driving speed and angle
        #rc.drive.set_speed_angle(speed, angle)

        # Set update_slow to refresh every half second
        rc.set_update_slow_time(0.5)
        # Have the car begin at a stop

        # Print start message
        print(">> Phase 1 Challenge: Cone Slaloming")

    def update_contour():
        """
        Finds contours in the current color image and uses them to update contour_center
        and contour_area
        """
        global contour_center
        global contour_area
        global cur_state
        global coneColor
        global contour_center


        image = rc.camera.get_color_image()

        if image is None:
            contour_center = None
            contour_area = 0
        else:

            image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

            # Find all of the orange and purple contours
            orangeContours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])      
            purpleContours = rc_utils.find_contours(image, PURPLE[0], PURPLE[1])

            # Select the largest orange and purple contour
            orangeContour = rc_utils.get_largest_contour(orangeContours, MIN_CONTOUR_AREA)
            purpleContour = rc_utils.get_largest_contour(purpleContours, MIN_CONTOUR_AREA)

            if orangeContour is None and purpleContour is None:
                contour = None
            elif orangeContour is None:
                contour = purpleContour
                coneColor = "purple"
            elif purpleContour is None:
                contour = orangeContour
                coneColor = "orange"
            else:
                if rc_utils.get_contour_area(orangeContour) > rc_utils.get_contour_area(purpleContour):
                    contour = orangeContour
                    coneColor = "orange"
                else:
                    contour = purpleContour
                    coneColor = "purple"

            if contour is not None:
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)

            else:
                contour_center = None
                contour_area = 0
            # Display the image to the screen
            rc.display.show_color_image(image)

    def purpleCurve(contour_center, contour_area):
        if contour_center is None:
            rc.drive.set_speed_angle(0.13, 0.11)
            print("autoPurpturn")
        else:
            TURN_ANGLE = ((contour_center[1] - 570) / 320)
            TURN_ANGLE = rc_utils.clamp(TURN_ANGLE, -1, 1)
            if -0.08 < TURN_ANGLE < 0.08:
                TURN_ANGLE = 0
            rc.drive.set_speed_angle(0.18, TURN_ANGLE)

    def orangeCurve(contour_center, contour_area):
        if contour_center is None:
            rc.drive.set_speed_angle(0.13, -0.11)
            print("autoOrangturn")
        else:
            TURN_ANGLE = ((contour_center[1] - 70) / 320)
            TURN_ANGLE = rc_utils.clamp(TURN_ANGLE, -1, 1)
            if -0.08 < TURN_ANGLE < 0.08:
                TURN_ANGLE = 0
            rc.drive.set_speed_angle(0.18, TURN_ANGLE)

    def update():

        global cur_state
        global queue
        global coneColor
        global contour_center
        global contour_area
        global queue

        update_contour()
        update_slow()
        print(cur_state)

        if coneColor == "orange":
            cur_state = State.orangeCurve
        elif coneColor == "purple":
            cur_state = State.purpleCurve

        if cur_state == State.orangeCurve:
            orangeCurve(contour_center, contour_area)
        elif cur_state == State.purpleCurve:
            purpleCurve(contour_center, contour_area)


    def update_slow():
        """
        After start() is run, this function is run at a constant rate that is slower
        than update().  By default, update_slow() is run once per second
        """
        # Print a line of ascii text denoting the contour area and x position
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
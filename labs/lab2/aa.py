# IMPORTS

import sys
import cv2 as cv
import numpy as np
import time

sys.path.insert(1, "../../library")

import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

import datetime

# START RACECAR

rc = racecar_core.create_racecar()

# CONSTANTS
MIN_CONTOUR_AREA = 200
PARKING_CONE_THRESHOLD = 100
CROP_FLOOR = ((120, 0), (rc.camera.get_height(), rc.camera.get_width()))

RED = ((165, 120, 160), (179, 255, 255)) # done (3, 182, 222), make it 10
GREEN = ((40, 50, 163), (88, 255, 255)) #done (78, 153, 213)
BLUE = ((90, 90, 140), (95, 255, 255))
#LIGHT_PURPLE = ((170, 20, 200), (179, 50, 255)) # done
PURPLE = ((135, 100, 150), (165, 255, 255)) # done
ORANGE = ((0, 160, 190), (10, 255, 255)) # done (10, 240, 240)
YELLOW = ((15, 40, 170), (32, 255, 255))   # done (26, 168, 228)

LINE_KP = 0.15
LINE_KI = 0
LINE_KD = 0.1

# WALL_KP = 0.0005
WALL_KP = 0.2
WALL_KI = 0
WALL_KD = 0.15

FRONT_WINDOW = (-5, 5)
REAR_WINDOW = (175, 185)
FRONT_RIGHT_WINDOW = (37.5, 52.5)
FRONT_LEFT_WINDOW = (305, 325)
RIGHT_WINDOW = (80, 100)

WALL_STOP_THRESHOLD = 125

class Color(IntEnum):
    GREEN = 0
    RED = 1
    ORANGE = 2
    PURPLE = 3
    BLUE = 4
    YELLOW = 5
    NONE = 6

class State(IntEnum):
    FOLLOW_LINE = 1
    CONE_SLALOM = 2
    FOLLOW_WALL = 3
    # BACK_UP = 4
    WALL_TURN = 5
    STOP = 6
    
# TRUE = CARPET, FALSE = NO CARPET
SPEEDS = {
    State.FOLLOW_LINE: {
        "LOW": 0.15,
        "HIGH": 0.15
    },
    State.CONE_SLALOM: {
        "LOW": 0.15,
        "MID": 0.15,
        "HIGH": 0.15
    },
    State.FOLLOW_WALL: {
        "LOW": 0.145,
        "HIGH": 0.145
    },
    State.WALL_TURN: {
        "LOW": 0.15,
        "HIGH": 0.155
    }
}
    
# GLOBAL VARIABLES

global speed
speed = 0.0
global angle
angle = 0.0
global contour_center
contour_center = None
global contour_area
contour_area = 0
global stop_time_counter
stop_time_counter = 0
global priority_color
priority_color = 0
global integral_sum
integral_sum = 0
global previous_error
previous_error = 0
global derivative
derivative = 0
global integral
integral = 0
global proportional
proportional = 0
global current_state
current_state = State.FOLLOW_LINE
global contours
contours = {}
global largest_contour
largest_contour = {}
global visible_colors
visible_colors = []
global last_color
last_color = 0
global has_been_switched
has_been_switched = False
global lidar_scan
lidar_scan = []
global lidar_points 
lidar_points = {
    "right": 0,
    "left": 0,
    "front": 0,
    "front_right": 0,
    "front_left": 0
}
global beginning_cone_slalom
beginning_cone_slalom = False
global cone_slalom_timer
cone_slalom_timer = 0
global markers
markers = [None, None]
global find_line
find_line=False
global new_error
new_error = 0
global wall_follow_timer
wall_follow_timer = 0
global cone_slalom_timer2
cone_slalom_timer2 = 0

# global contour_dimensions
# contour_dimensions = (0, 0, 0, 0)

# FUNCTIONS
    
def start():
    global speed
    global angle
    global current_state
    global integral_sum
    global beginning_cone_slalom
    global wall_follow_timer
    global cone_slalom_timer2
    
    speed = 0
    angle = 0
    integral_sum = 0
    beginning_cone_slalom = False
    wall_follow_timer = 0
    cone_slalom_timer2 = 0
    
    current_state = State.FOLLOW_LINE
    
    
    # set initial speed and angle to 0
    rc.drive.set_speed_angle(speed, angle)
    # the update_slow() function will be called every 0.5 seconds
    rc.set_update_slow_time(1)
    
    # start program message
    print(
        "racetime"
    )

def update():
    global current_state
    global stop_time_counter
    global integral_sum
    global previous_error
    global WALL_KD
    global beginning_cone_slalom
    ar_scan()
    if current_state == State.STOP:
        stop_car()
    elif current_state == State.FOLLOW_LINE:
        update_contour()
        follow_line()
    elif current_state == State.CONE_SLALOM:
        if stop_time_counter < 3:
            beginning_cone_slalom = True
            stop_car()
        else:
            cone_slalom()
    elif current_state == State.FOLLOW_WALL:
        follow_wall()
    elif current_state == State.WALL_TURN:
        turn_on_wall()
    if rc.controller.was_pressed(rc.controller.Button.X):
        integral_sum = 0
        previous_error = 0
        current_state = State.FOLLOW_WALL

def turn_on_wall():
    global lidar_scan
    global speed
    global angle 
    global current_state
    global lidar_points
    
    lidar_scan = rc.lidar.get_samples()    
    kernel = [0.25, 0.5, 0.25]
    lidar_scan = convolution(lidar_scan, kernel)
    
    speeds = SPEEDS[current_state]
    
    # lower speeds on sharper turns, higher speeds on straighter paths
    if abs(angle) > LINE_KP / 2:    #so wall kp is super tiny so i made it use line kp TODO: TO BE EDITED
        speed = speeds["LOW"]
    else:
        speed = speeds["HIGH"]
    lidar_points["front"] = rc_utils.get_lidar_average_distance(lidar_scan, FRONT_WINDOW[0], FRONT_WINDOW[1])
    lidar_points["front_right"] = rc_utils.get_lidar_average_distance(lidar_scan, RIGHT_WINDOW[0], RIGHT_WINDOW[1])
    lidar_points["front_left"] = rc_utils.get_lidar_average_distance(lidar_scan, FRONT_LEFT_WINDOW[0], FRONT_LEFT_WINDOW[1])
    if lidar_points["front"] > 50:
        current_state = State.FOLLOW_WALL
    angle = (lidar_points["front_right"] - lidar_points["front_left"]) * WALL_KP
    if angle > 1:
        angle = 0.75
    elif angle < -1:
        angle = -0.75
    # set the speed and angle!
    rc.drive.set_speed_angle(speed, angle)

# def back_up():
#     global lidar_scan
#     global speed
#     global angle 
#     global current_state
#     speed=-0.2
#     angle=0
#     lidar_scan = rc.lidar.get_samples()    
#     kernel = [0.25, 0.5, 0.25]
#     lidar_scan = convolution(lidar_scan, kernel)
#     forward_dist = rc_utils.get_lidar_closest_point(lidar_scan, FRONT_WINDOW)
#     if forward_dist[1] > 100: # change the number until it works idk man
#         current_state = State.FOLLOW_WALL
#     rc.drive.set_speed_angle(speed, angle)

def stop_car():
    global stop_time_counter
    
    if stop_time_counter > 3:
        rc.drive.stop()
    else:
        rc.drive.set_speed_angle(-1, 0)

    stop_time_counter += rc.get_delta_time()

def follow_line():
    global speed
    global angle
    
    if contour_center is not None:
        add_pid(LINE_KP, LINE_KI, LINE_KD)
        
    speeds = SPEEDS[current_state]
    # lower speeds on sharper turns, higher speeds on straighter paths
    if abs(angle) > LINE_KP / 2:
        speed = speeds["LOW"]
    else:
        speed = speeds["HIGH"]

    # set the speed and angle!
    rc.drive.set_speed_angle(speed, angle)

def cone_slalom():
    global speed
    global angle
    global priority_color
    global last_color
    global current_state
    global stop_time_counter
    global has_been_switched
    global cone_slalom_timer
    global cone_slalom_timer2
    
    cone_slalom_timer2 += rc.get_delta_time()
    
    if cone_slalom_timer2 < 1.5:
        rc.drive.set_speed_angle(0.14, 0.1)
    else:
        if priority_color == Color.ORANGE:
            speed = SPEEDS[current_state]["HIGH"]
            angle = 0.12
            has_been_switched = False
        elif priority_color == Color.PURPLE:
            speed = SPEEDS[current_state]["HIGH"]
            angle = -0.12
            has_been_switched = False
        elif priority_color == Color.NONE and not has_been_switched:
            cone_slalom_timer += rc.get_delta_time()
            if cone_slalom_timer > 1:
                speed = SPEEDS[current_state]["MID"]
                cone_slalom_timer = 0
            else:
                speed = SPEEDS[current_state]["LOW"]                
            angle *= -1
            has_been_switched = True
        if priority_color == Color.RED:
            current_state = State.STOP
        update_cone_contour()
        rc.drive.set_speed_angle(speed,angle)

def convolution(array: np.ndarray, kernel: list) -> np.ndarray:
    kernel = kernel[::-1]
    end_array = np.zeros(array.size)
    array = np.pad(array, (len(kernel)//2, len(kernel)//2), 'constant')
    for i in range(array.size-((len(kernel)//2) *2)):
        end_array[i] = np.sum(array[i:i+len(kernel)] * kernel)
    end_array = end_array[1:-1]
    return end_array

def ar_scan():
    global current_state
    global markers
    global find_line
    global wall_follow_timer
    
    image = rc.camera.get_color_image()
    image = image[10:120]
    markers = cv.aruco.detectMarkers(
        image,
        cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
        parameters=cv.aruco.DetectorParameters_create()
    )
    
    if markers is not None:
        if markers[1] is not None:
            if markers[1][0][0] == 4:
                wall_follow_timer = 0
                current_state = State.FOLLOW_WALL
                SPEEDS[State.FOLLOW_LINE] = {
                    "LOW": 0.1225,
                    "HIGH": 0.125                   
                }
            elif markers[1][0][0] == 0:
                find_line = True
            elif markers[1][0][0] == 1:
                current_state = State.CONE_SLALOM
            elif markers[1][0][0] == 3:
                current_state = State.STOP

def follow_wall():
    global lidar_scan
    global speed
    global angle 
    global current_state
    global lidar_points
    global find_line
    global wall_follow_timer
    
    wall_follow_timer += rc.get_delta_time()
    if wall_follow_timer < 1.5:
        if wall_follow_timer < 0.75:
            rc.drive.set_speed_angle(SPEEDS[current_state]["LOW"], 0.3)
        else:
            rc.drive.set_speed_angle(SPEEDS[current_state]["LOW"], -0.3)
    else:
        lidar_scan = rc.lidar.get_samples()    
        kernel = [0.25, 0.5, 0.25]
        lidar_scan = convolution(lidar_scan, kernel)
        add_pid(WALL_KP, WALL_KI, WALL_KD)
        
        speeds = SPEEDS[current_state]
        
        # lower speeds on sharper turns, higher speeds on straighter paths
        speed = speeds["HIGH"]
        # if abs(angle) > LINE_KP / 2:
        #     speed = speeds["LOW"]
        # else:
            
        if find_line:
            update_contour()
        
        #TODO bring turn wall back if needed
        # lidar_points["front"] = rc_utils.get_lidar_closest_point(lidar_scan, FRONT_WINDOW)
        # if lidar_points["front"][1] < 50:
        #     current_state = State.WALL_TURN
        # elif lidar_points["front"][1] < 50:
        #     current_state = State.WALL_TURN

        # set the speed and angle!
        rc.drive.set_speed_angle(speed, angle)
        
def stop_at_wall():
    global current_state
    global lidar_points
    
    scan = rc.lidar.get_samples()
    lidar_points["front"] = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    if lidar_points["front"][1] < WALL_STOP_THRESHOLD:
        current_state = State.STOP

def add_pid(kp, ki, kd):
    global angle
    global contour_center
    global integral_sum
    global previous_error
    global derivative
    global integral
    global proportional
    global current_state
    global lidar_scan
    global lidar_points
    # global contour_dimensions
    global speed
    global new_error
    
    new_error = 0
    
    if current_state == State.FOLLOW_LINE:
        new_error = contour_center[1] / 320 * 2 - 1
    elif current_state == State.FOLLOW_WALL:
        lidar_points["right"] = rc_utils.get_lidar_average_distance(lidar_scan, RIGHT_WINDOW[0], RIGHT_WINDOW[1])
        #lidar_points["right"] = rc_utils.get_lidar_average_distance(lidar_scan, FRONT_RIGHT_WINDOW

        # lidar_points["left"] = rc_utils.get_lidar_average_distance(lidar_scan, 295, 25)
        # magnitude = lidar_points["left"] + lidar_points["right"]
        # greater than 50, turn right
        # less than 50, turn left

        #new_error = (lidar_points["right"] - 25) / 50
        new_error = (lidar_points["right"] - 60) / 20
        
    proportional = new_error * kp
    
    dt = rc.get_delta_time()

    integral_sum += (previous_error + new_error) / 2 * dt
    integral = integral_sum * ki
    
    derivative = ((new_error - previous_error) / dt) * kd

    previous_error = new_error
    
    angle = proportional + integral + derivative
    
    if angle > 1:
        angle = 1
    elif angle < -1:
        angle = -1
        
def update_contour():
    global contour_center
    global contour_area
    global priority_color
    global stop_time_counter
    global current_state
    global contours
    global largest_contour
    global visible_colors
    global angle
    global last_color
    # global contour_dimensions
    
    image = rc.camera.get_color_image()
    
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # crop image 140 pixels from the top and 20 pixels from the bottom
        image = image[140:220]
        
        # get all color contours
        contours = get_all_color_contours(image)
        
        # get largest contour for each color contour
        largest_contours = get_all_color_largest_contours(contours)
        
        # add the color if it is visible to the RACECAR
        visible_colors = get_all_color_visible_colors(largest_contours)
        #print(visible_colors)
                
        # get the highest priority color
        last_color = priority_color
        priority_color = get_priority_color(visible_colors)

        # if priority_color == Color.ORANGE and current_state == State.FOLLOW_LINE:
        #     contour_center = rc_utils.get_contour_center(largest_contours[Color.ORANGE])
        #     contour_area = rc_utils.get_contour_area(largest_contours[Color.ORANGE])
        #     stop_time_counter = 0
        #     current_state = State.CONE_SLALOM
        #     angle = 0.05
        if priority_color == Color.GREEN:
            current_state = State.FOLLOW_LINE
            contour_center = rc_utils.get_contour_center(largest_contours[Color.GREEN])
            contour_area = rc_utils.get_contour_area(largest_contours[Color.GREEN])
            stop_time_counter = 0
        elif priority_color == Color.RED:
            contour_center = rc_utils.get_contour_center(largest_contours[Color.RED])
            contour_area = rc_utils.get_contour_area(largest_contours[Color.RED])
            stop_time_counter = 0
        elif priority_color == Color.BLUE:
            contour_center = rc_utils.get_contour_center(largest_contours[Color.BLUE])
            contour_area = rc_utils.get_contour_area(largest_contours[Color.BLUE])
            stop_time_counter = 0
        elif priority_color == Color.YELLOW:
            contour_center = rc_utils.get_contour_center(largest_contours[Color.YELLOW])
            contour_area = rc_utils.get_contour_area(largest_contours[Color.YELLOW])
            stop_time_counter = 0
        # if priority_color != Color.NONE:
            # contour_dimensions = cv.boundingRect(largest_contours[priority_color])

def update_cone_contour():
    global contour_center
    global contour_area
    global priority_color
    global stop_time_counter
    global current_state
    global contours
    global largest_contour
    global visible_colors
    global last_color
    
    image = rc.camera.get_color_image()
    
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # crop image 140 pixels from the top and 20 pixels from the bottom
        image = image[140:220]
        
        # get all color contours
        contours = get_all_cone_contours(image)
        
        # get largest contour for each color contour
        largest_contours = get_all_cone_largest_contours(contours)
        
        # add the color if it is visible to the RACECAR
        visible_colors = get_all_cone_visible_colors(largest_contours)
        
        # get the highest priority color
        last_color = priority_color
        priority_color = get_priority_color(visible_colors)
        
def get_all_color_contours(image):
    return {
        # "yellow": rc_utils.find_contours(image, YELLOW[0], YELLOW[1]),
        Color.GREEN: rc_utils.find_contours(image, GREEN[0], GREEN[1]),
        Color.RED: rc_utils.find_contours(image, RED[0], RED[1]),
        Color.BLUE: rc_utils.find_contours(image, BLUE[0], BLUE[1]),
        Color.YELLOW: rc_utils.find_contours(image, YELLOW[0], YELLOW[1]),
    }

def get_all_cone_contours(image):
    return {
        Color.ORANGE: rc_utils.find_contours(image, ORANGE[0], ORANGE[1]),
        Color.PURPLE: rc_utils.find_contours(image, PURPLE[0], PURPLE[1]),
        Color.BLUE: rc_utils.find_contours(image, BLUE[0], BLUE[1])
    }

def get_all_color_largest_contours(contours):
    return {
        # "yellow": rc_utils.get_largest_contour(contours["yellow"], MIN_CONTOUR_AREA),
        Color.GREEN: rc_utils.get_largest_contour(contours[Color.GREEN], MIN_CONTOUR_AREA),
        Color.RED: rc_utils.get_largest_contour(contours[Color.RED], MIN_CONTOUR_AREA),
        Color.BLUE: rc_utils.get_largest_contour(contours[Color.BLUE], MIN_CONTOUR_AREA),
        Color.YELLOW: rc_utils.get_largest_contour(contours[Color.YELLOW], MIN_CONTOUR_AREA),
    }

def get_all_cone_largest_contours(contours):
    return {
        Color.ORANGE: rc_utils.get_largest_contour(contours[Color.ORANGE], MIN_CONTOUR_AREA),
        Color.PURPLE: rc_utils.get_largest_contour(contours[Color.PURPLE], MIN_CONTOUR_AREA),
        Color.BLUE: rc_utils.get_largest_contour(contours[Color.BLUE], MIN_CONTOUR_AREA),
    }

def get_all_color_visible_colors(largest_contours):
    result = []
    
    if largest_contours[Color.YELLOW] is not None:
        result.append(Color.YELLOW)
    if largest_contours[Color.BLUE] is not None:
        result.append(Color.BLUE)
    if largest_contours[Color.RED] is not None:
        result.append(Color.RED)
    if largest_contours[Color.GREEN] is not None:
        result.append(Color.GREEN)

    return result

def get_all_cone_visible_colors(largest_contours):
    result = []
    
    if largest_contours[Color.ORANGE] is not None:
        result.append(Color.ORANGE)
    if largest_contours[Color.PURPLE] is not None:
        result.append(Color.PURPLE)
    if largest_contours[Color.BLUE] is not None:
        result.append(Color.BLUE)   

    return result

def get_priority_color(visible_colors):
    priority_color = Color.NONE

    for color in visible_colors:
        if priority_color > color:
            priority_color = color
    
    return priority_color
        
def update_slow():
    global contour_area
    global angle
    global speed
    global priority_color
    global contour_center
    global contour_area
    global current_state
    global derivative
    global integral
    global proportional
    global lidar_points
    global markers
    global new_error
    # global contour_dimensions
    
    if rc.camera.get_color_image() is None:
        print("No image")
    else:

        print("state", current_state)
        print(new_error)
        if markers[1] is not None:
               print("marker: ", markers[1][0][0])




# DO NOT MODIFY!
if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
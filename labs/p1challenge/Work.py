"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
"""

# Imports
#region
from enum import IntEnum
import sys
import cv2 as cv
from matplotlib import pyplot as plt
import numpy as np
import constants
sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
#endregion

# States for state machine

class States(IntEnum):
    Blue = -1
    Red = 1
    Turn = 2
    Line_Follow = 3



# Generic global variables
global current_state, last_state

current_state = States.Line_Follow
last_state = None

# Line Follow global variables
global rc, color_queue_timer, color_queue_index, integral_sum, PID_timer, last_error

color_queue = (
    (constants.BLUE_LINE, constants.ORANGE_LINE), 
    (constants.GREEN_LINE, constants.ORANGE_LINE), 
    (constants.RED_LINE, constants.BLUE_LINE), 
    (constants.BLUE_LINE, constants.ORANGE_LINE)
)
color_queue_index = 0
color_queue_timer = 0

integral_sum = 0.0
PID_timer = 0.001
last_error = 0.0

# Cone Slalom global variables
global slalom_timer

slalom_timer = 0

rc = racecar_core.create_racecar()

"""
This function is run once every time the start button is pressed
"""
def start():
    rc.drive.stop()

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

"""
After start() is run, this function is run every frame until the back button
is pressed
"""
def update():
    # TODO: Slalom between red and blue cones.  The car should pass to the right of
    # each red cone and the left of each blue cone.
    global slalom_timer, PID_timer
    speed, angle  = 0, 0

    if(current_state == States.Blue):
        speed, angle = cone_search(States.Blue, constants.CS_BLUE_CONE_OFFSET, constants.BLUE_CONE)
    elif(current_state == States.Red):
        speed, angle = cone_search(States.Red, constants.CS_RED_CONE_OFFSET, constants.RED_CONE)
    elif(current_state == States.Turn):
        speed, angle = turn()
    elif(current_state == States.Line_Follow):
        speed, angle = line_follow()

    PID_timer += rc.get_delta_time()
    slalom_timer += rc.get_delta_time()
    rc.drive.set_speed_angle(speed, angle)

# Line Follow functions

def line_follow():
    global color_queue_index, color_queue_timer
    image = rc.camera.get_color_image()
    cropped_image = rc_utils.crop(image, constants.LINE_FOLLOW_IMG_CROP[0], constants.LINE_FOLLOW_IMG_CROP[1])

    # Initial speed and angle values will be set to safe value
    speed, angle = constants.DEFAULT_SAFE_SPEED, 0

    # Gets info about the current color contour and the next color contour in the color queue
    current_contour_center, current_contour_area = find_contours(color_queue[color_queue_index][0], cropped_image, False)
    next_contour_center, next_contour_area = find_contours(color_queue[color_queue_index][1], cropped_image, False)

    # Returns the safe values when the current contour is none
    if(current_contour_center is None):
        return speed, angle

    # If the timer is not 1 and the next contour is found
    if color_queue_timer <= 0 and next_contour_area > 1000:
        print("next found")
        # The color_queue_index index is incremented so that we can look for the next color pair
        color_queue_index += 1
        # The timer is set 1, cooldown timer so that the camera doesnt immediatley switch colors
        color_queue_timer = 2
        # The current values are set to the next values
        current_contour_center, current_contour_area = next_contour_center, next_contour_area

    # The angle is returned based off of the x value of the current contour center
    angle = get_controller_output(current_contour_center[1])

    # Decrements the color_queue_timer by the delta time so that we know when 1 second has passed
    if color_queue_timer > 0:
        color_queue_timer -= rc.get_delta_time()
    faster_crop = rc_utils.crop(image, constants.LINE_STRAIGHT_IMG_CROP[0], constants.LINE_STRAIGHT_IMG_CROP[1])
    # Checks if car is on long straight-away, and returns faster speed
    long_contour_center, long_contour_area = find_contours(color_queue[color_queue_index][0], cropped_image, False)
    if long_contour_area > 1000:
        return constants.LINE_STRAIGHT_SPEED, angle
    # The speed is a constant set above, and the angle is returned from the controller
    return constants.LINE_FOLLOWING_SPEED, angle

# Cone Slalom functions

# region
def cone_search(state, offset, color):
    global current_state, last_state, slalom_timer

    # Gets contour center and area from the image
    image = rc.camera.get_color_image()
    cropped_image = rc_utils.crop(image, constants.CS_IMG_CROP[0], constants.CS_IMG_CROP[1])
    contour_center, contour_area = find_contours(color, cropped_image, False)

    # Goes straight at the cone center + offset for 2.5 seconds
    if(slalom_timer <= constants.CS_STRAIGHT and contour_center is not None):
        angle_px = contour_center[1] + offset
        angle_rc = angle_px * (2 / 360) - 1
        return constants.CS_SPEED, clamp(angle_rc, -1, 1)

    # If it can still see the cone and 2.5 seconds is up, is keeps going straight
    if(contour_area >= 0):
        return constants.CS_SPEED, 0
    
    # If it can't see the cone and 2.5 seconds is up then it switches to the turn state
    # Sets timer to 0 so that hte same timer can be used for the turn() function
    slalom_timer = 0
    last_state = state
    current_state = States.Turn
    # Have to return 0,0 since the state machine requires a value. The state will only be switched next frame
    return 0, 0

def turn():
    global current_state, last_state, slalom_timer
    # Gets the contour area, center and the next state from the find_cone() function
    # next state lets us set the next state machine set at the end of this funciton depending on
    # the color of the biggest contour seen. Lets us keep going even if there are 2 cones of the same
    # color in a line
    contour_center, contour_area, next_state = find_cone()
    # Sets the angle to be negative of the last state value. The last_state int value is representative
    # of the side it should turn, so the negative value will let us turn around the cone
    angle = -last_state

    # First goes straight for 1.5 seconds. This is to account for the cone width. Since the fuction is called
    # right after the cone is not able to be seen, this lets us make sure that we won't hit the cone when we turn
    if(slalom_timer < 1.5):
        return constants.CS_SPEED, 0
    
    # Once 
    if((contour_area == -1 or contour_center == None) or slalom_timer < constants.CS_TURN):
        return constants.CS_SPEED, angle

    slalom_timer = 0
    last_state = States.Turn
    current_state = next_state
    return 0, 0
    

def find_cone():
    image = rc.camera.get_color_image()
    cropped_image = rc_utils.crop(image, constants.CS_IMG_CROP[0], constants.CS_IMG_CROP[1])

    red_contour_center, red_contour_area = find_contours(constants.RED_CONE, cropped_image, True)
    blue_contour_center, blue_contour_area = find_contours(constants.BLUE_CONE, cropped_image, True)
    countour_center, contour_area, state = None, -1, States.Turn

    if red_contour_area > blue_contour_area:
        countour_center = red_contour_center
        contour_area = red_contour_area
        state = States.Red
    elif(blue_contour_area >= red_contour_area):
        countour_center = blue_contour_center
        contour_area = blue_contour_area
        state = States.Blue

    return countour_center, contour_area, state

# endregion

def find_contours(color, image, show):    
    # When the image is not found: None, None is returned
    if image is None:
        # print("No Image")
        return None, None
        
    # Find all contours of given color
    list_contours = rc_utils.find_contours(image, color[0], color[1])

    # Gets the largest contour
    contour = rc_utils.get_largest_contour(list_contours, constants.CS_MIN_CONTOUR)

    # If no contour was found: None, None is returned
    if(contour is None):
        # print("No contour found")
        return None, -1

    # Gets information about the center and area
    contour_center = rc_utils.get_contour_center(contour)
    contour_area = rc_utils.get_contour_area(contour)

    if(show):
        rc_utils.draw_contour(image, contour)
        rc_utils.draw_circle(image, contour_center)
        
        rc.display.show_color_image(image)

    # Returns the largest contour center and area
    return contour_center, contour_area

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

def get_controller_output(center):
    global integral_sum, last_error, PID_timer
    # ranges:
    # center:  0 - 320
    # error: -160 - +160
    # output_rc: -1 - 1
    error = center - (constants.WIDTH / 2)
    integral_sum += PID_timer * error
    slope = error - last_error / PID_timer    
    output_px = (constants.LINE_FOLLOW_kP * (error) + (constants.LINE_FOLLOW_kI * integral_sum) + (constants.LINE_FOLLOW_kD * slope))

    output_rc = output_px / (constants.WIDTH / 2)
    last_error, PID_timer = error, 0
    return clamp(output_rc, -1, 1)


def switch_between_color_depth(center):
    newCoords = (int(center[0]), int(center[1]))
    return newCoords
    # 320,240
    # 160,120


def search_for_depth(center):
    # creates the depth image
    depth_image = rc.camera.get_depth_image()

    # gets the distance of the center pixel within the depth image (we will assume it is off a cone)
    center_distance = depth_image[center[0]][center[1]]
    print(center_distance)
    # checks if the center distance is under 30
    if center_distance < 20:
        return True
    else:
        return False

    '''
    Gets the distance of the center pixel, and if it is under an x amount of cm then it is set to true
    '''


def check_cone_contour(color, crop, threshold):
    center, area = find_contours(color, crop)

    if area is not None and area > threshold:
        print("FOR ONCE")
        return True
    else:
        print("failed")

    '''
    Check the color of the center pixel of the image, once the pixel is seen that it is within the range (HSV)
    '''

def final_follow(COLOR,CROP):

    speed, angle = DEFAULT_SAFE_SPEED, 0

    contour_center, contour_area = find_contours(COLOR, CROP)

    if (contour_center is None):
        print("dead")
        return speed, angle
    print("COLOR FOUND")
    angle = get_controller_output(contour_center[1])

    return DEFAULT_SAFE_SPEED, angle

def center_finder(COLOR,CROP):
    contour_center, contour_area = find_contours(COLOR, CROP)

    if (contour_center is None):
        return 0

    return contour_center

# DO NOT MODIFY: Register start and update and begin execution
if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
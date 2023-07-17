"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2A - Color Image Line Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
import math
########################################################################################
# Global variables
########################################################################################
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum
rc = racecar_core.create_racecar()
DRIVE   = True
# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (120, 255, 255))  # The HSV range for the color blue
# TODO (challenge 1): add HSV ranges for other colors

# >> Variables
speed = 0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
image = None
########################################################################################
# Functions
########################################################################################



class states(Enum):
    following_line = 1
    parking_cone = 2
    stop = 0



def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
) -> float:
    """
    Remaps a value from one range to another range.

    Args:
        val: A number form the old range to be rescaled.
        old_min: The inclusive 'lower' bound of the old range.
        old_max: The inclusive 'upper' bound of the old range.
        new_min: The inclusive 'lower' bound of the new range.
        new_max: The inclusive 'upper' bound of the new range.

    Note:
        min need not be less than max; flipping the direction will cause the sign of
        the mapping to flip.  val does not have to be between old_min and old_max.
    """
    # TODO: remap val to the new range
    a = (val - old_min) / (old_max - old_min)
    return a * (new_max - new_min) + new_min
def clamp(value: float, vmin: float, vmax: float) -> float:
    """
    Clamps a value between a minimum and maximum value.

    Args:
        value: The input to clamp.
        vmin: The minimum allowed value.
        vmax: The maximum allowed value.

    Returns:
        The value saturated between vmin and vmax.
    """
    # TODO: Make sure that value is between min and max
    if value < vmin:
        value = vmin
    elif value > vmax:
        value = vmax
    return value
def get_mask(
    image: NDArray[(Any, Any, 3), np.uint8],
    hsv_lower: Tuple[int, int, int],
    hsv_upper: Tuple[int, int, int]
) -> NDArray[Any, Any]:
    """   
    Returns a mask containing all of the areas of image which were between hsv_lower and hsv_upper.
    
    Args:
        image: The image (stored in BGR) from which to create a mask.
        hsv_lower: The lower bound of HSV values to include in the mask.
        hsv_upper: The upper bound of HSV values to include in the mask.
    """
    # Convert hsv_lower and hsv_upper to numpy arrays so they can be used by OpenCV
    hsv_lower = np.array(hsv_lower)
    hsv_upper = np.array(hsv_upper)
    
    # TODO: Use the cv.cvtColor function to switch our BGR colors to HSV colors
    img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    
    # TODO: Use the cv.inRange function to highlight areas in the correct range
    mask = cv.inRange(img, hsv_lower, hsv_upper)
    
    return mask
def find_contours(mask: NDArray) -> List[NDArray]:
    """
    Returns a list of contours around all objects in a mask.
    """
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]
def get_largest_contour(contours: List[NDArray], min_area: int = 10) -> Optional[NDArray]:
    """
    Finds the largest contour with size greater than min_area.

    Args:
        contours: A list of contours found in an image.
        min_area: The smallest contour to consider (in number of pixels)

    Returns:
        The largest contour from the list, or None if no contour was larger than min_area.
    """
    if len(contours) == 0:
        # TODO: What should we return if the list of contours is empty?
        return None
    
    # TODO: Return the largest contour, but return None if no contour is larger than min_area
    max = 0
    
    for i in contours:
        if cv.contourArea(i) > max:
            max = cv.contourArea(i) 
            contour = i
    if max <= min_area:
        return None
    return contour

def get_contour_center(contour: NDArray) -> Optional[Tuple[int, int]]:
    """
    Finds the center of a contour from an image.

    Args:
        contour: The contour of which to find the center.

    Returns:
        The (row, column) of the pixel at the center of the contour, or None if the contour is empty.
    """
    # Ask OpenCV to calculate the contour's moments
    M = cv.moments(contour)

    # Check that the contour is not empty
    if M["m00"] <= 0:
        return None

    # Compute the center of mass of the contour
    center_row = round(M["m01"] / M["m00"])
    center_column = round(M["m10"] / M["m00"])
    
    return (center_row, center_column)
cone_area = 0
def update_contour():
    global DRIVE
    global image
    global cone_area
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global angle
    global current_state
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)

        # Crop the image to the floor directly in front of the car
        image = image[150:len(image)]
        colors = {
            "blue":[(90, 50, 50),(115, 200, 255)],
            "red":[(0, 50, 50), (10, 255, 255)],
            "green":[(40, 50, 50), (70, 255, 255)],
            "cone":[(140,95,95), (155,240,240)]
        }

        hsv = [
            colors["green"],
            colors["red"],
            colors["blue"],
            colors["cone"]
        ]

        image_copy = np.copy(image)


        cone_mask = get_mask(image_copy, hsv[3][0], hsv[3][0])
        cone_contours = find_contours(cone_mask)
        if len(cone_contours) >0:
            
            largest_contour = get_largest_contour(cone_contours)
            contour_center = get_contour_center(largest_contour)
            try:
                cone_area = cv.contourArea(largest_contour)
                current_state = states.parking_cone
            except:
                pass
            return
        first_mask = get_mask(image, hsv[0][0], hsv[0][1])
        first_contours = find_contours(first_mask)
        if len(first_contours)>0:
            largest_contour = get_largest_contour(first_contours)
            contour_center = get_contour_center(largest_contour)
            try:
                contour_area = cv.contourArea(largest_contour)
            except:
                pass
            return
        else:
            second_mask = get_mask(image, hsv[1][0], hsv[1][1])
            second_contours = find_contours(second_mask)
            if len(second_contours)>0:
                largest_contour = get_largest_contour(second_contours)
                contour_center = get_contour_center(largest_contour)
                try:
                    contour_area = cv.contourArea(largest_contour)
                except:
                    pass
                return
            else:
                final_mask = get_mask(image, hsv[2][0], hsv[2][1])
                final_contours = find_contours(final_mask)
                if len(final_contours)>0:
                    largest_contour = get_largest_contour(final_contours)
                    contour_center = get_contour_center(largest_contour)
                    try:
                        contour_area = cv.contourArea(largest_contour)
                    except:
                        pass
                    return
                else:
                    contour_center = None
                    contour_area = 0



        # Display the image to the screen
def pid_control(
    p_gain,
    i_gain,
    d_gain,
    target_val,
    current_val,
    accumulated_error,
    last_error,
    dt
):

    error = target_val - current_val
    #change dt with time of car
    # Update the accumulated error
    accumulated_error += error * dt


    delta_error = (error - last_error) / dt

    p_term = p_gain * error
    i_term = i_gain * accumulated_error
    d_term = d_gain * delta_error

    return p_term + i_term + d_term, accumulated_error, error

def clamp(value: float, min_value: float, max_value: float) -> float:

    return max(min(value, max_value), min_value)

target_val = 160
current_val = 0.0
accumulated_error = 0.0
last_error = 0.0
dt = 1.0

current_state = 1

def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle

    # Initialize variables
    speed = 1
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    A button = print current speed and angle\n"
        "    B button = print contour center and area"
    )
PID_P = 3
PID_I = 1.5
PID_D = 0.7
def update():
    global PID_P,PID_I,PID_D
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global current_val,accumulated_error,last_error,dt,current_state
    dt=rc.get_delta_time()
    update_contour()
    if current_state==1:
        if contour_center is not None:
            # Current implementation: bang-bang control (very choppy)
            # TODO (warmup): Implement a smoother way to follow the line
    #         pos = remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
    #         angle= clamp(pos, -1, 1)+0.5
            
            anglea,accumulated_error,last_error = pid_control(PID_P, PID_I, PID_D, 320, contour_center[1], accumulated_error, last_error, dt)
            angle = remap_range(anglea, -640,640, 1, -1)
            angle= clamp(angle, -1, 1)
        rc.drive.set_speed_angle(speed, angle)
    
    if current_state == states.parking_cone:
        if cone_area < 500:
            rc.drive.set_speed_angle(speed, 0)
        else:
            rc.drive.stop()
            current_state=states.stop
    
    if rc.controller.was_pressed(rc.controller.Button.X):
        speed+=0.01

    if rc.controller.was_pressed(rc.controller.Button.Y):
        speed-=0.01

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle, "PID: ",PID_P,PID_I,PID_D)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
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

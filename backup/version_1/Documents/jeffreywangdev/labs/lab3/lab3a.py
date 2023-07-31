"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""
import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum
########################################################################################
# Imports
########################################################################################
def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
) -> float:

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

def generate_gaussian_filter(sigma: int,filter_shape: list):
    # 'sigma' is the standard deviation of the gaussian distribution

    m, n = filter_shape
    m_half = m // 2
    n_half = n // 2

    # initializing the filter
    gaussian_filter = np.zeros((m, n), np.float32)

    # generating the filter
    for y in range(-m_half, m_half):
        for x in range(-n_half, n_half):
            normal = 1 / (2.0 * np.pi * sigma**2.0)
            exp_term = np.exp(-(x**2.0 + y**2.0) / (2.0 * sigma**2.0))
            gaussian_filter[y+m_half, x+n_half] = normal * exp_term
    return gaussian_filter
def convolution(image: np.ndarray, kernel: list ) -> np.ndarray:

    if len(image.shape) == 3:
        m_i, n_i, c_i = image.shape

    # if the image is gray then we won't be having an extra channel so handling it
    elif len(image.shape) == 2:
        image = image[..., np.newaxis]
        m_i, n_i, c_i = image.shape
    else:
        raise Exception('Shape of image not supported')

    m_k, n_k = kernel.shape

    y_strides = m_i - m_k + 1  # possible number of strides in y direction
    x_strides = n_i - n_k + 1  # possible number of strides in x direction

    img = image.copy()
    output_shape = (m_i-m_k+1, n_i-n_k+1, c_i)
    output = np.zeros(output_shape, dtype=np.float32)

    count = 0  # taking count of the convolution operation being happening

    output_tmp = output.reshape(
        (output_shape[0]*output_shape[1], output_shape[2])
    )

    for i in range(y_strides):
        for j in range(x_strides):
            for c in range(c_i): # looping over the all channels
                sub_matrix = img[i:i+m_k, j:j+n_k, c]

                output_tmp[count, c] = np.sum(sub_matrix * kernel)

            count += 1

    output = output_tmp.reshape(output_shape)

    return output
def get_average(img):
    a = 0
    b = 0
    for i in img:
        for j in i:
            a+=1
            b+=j
    if a == 0:
        return 0
    return b/a
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
global kernal 
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelxerate forward\n"
        "    Right bumper = override safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print the distance at the center of the depth image"
    )

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global kernal
    image = rc.camera.get_color_image()
    cone_distance = 1000
    if image is not None:
    
        cone_mask = get_mask(image, (120, 80, 80), (160,155,200))
        cone_contours = find_contours(cone_mask)
        if len(cone_contours) >0:
            depth_img = rc.camera.get_depth_image()
            try:
                largest_contour = get_largest_contour(cone_contours)
                contour_center = get_contour_center(largest_contour)
                cone_area = cv.contourArea(largest_contour)
                x = int(remap_range(contour_center[0],0,len(image),0,len(depth_img[0])))
                y = int(remap_range(contour_center[1],0,len(image[1]),0,len(depth_img[1])))
                dp_img = depth_img[x-15:x+15,y-15:y+15]
                cone_distance = get_average(dp_img)
                print(cone_distance)
            except:
                pass
    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Calculate the distance of the object directly in front of the car
    center_distance = cone_distance
    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    # Define the desired distance from the cone (in meters)
    center_distance = center_distance-100
    if center_distance > 1000:
        center_distance = 1000
    if center_distance<0:
        center_distance = 0
    # Calculate the difference between the desired distance and the actual distance
    speed = remap_range(center_distance,0,1000,-0.5,0.15)



    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # Display the current depth image



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

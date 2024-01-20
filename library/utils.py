from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

def draw_contour(
    image: NDArray,
    contour: NDArray,
    color: Tuple[int, int, int] = (0, 255, 0)
) -> None:
    """
    Draws a contour on the provided image.

    Args:
        image: The image on which to draw the contour.
        contour: The contour to draw on the image.
        color: The color to draw the contour in BGR format.
    """
    if contour is not None:
        cv.drawContours(image, [contour], 0, color, 3)
    return image
        # Display the image to the screen

def show_image(image: NDArray) -> None:
    """
    Displays a color image in the Jupyter Notebook.
    """
    plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
    plt.show()
    
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

def average(lista):
    return sum(lista)/len(lista)

def get_v(v0, a, t):
    return v0 + a*t

def convolution(array: np.ndarray, kernel: list) -> np.ndarray:
    kernel = kernel[::-1]
    end_array = np.zeros(array.size)
    array = np.pad(array, (len(kernel)//2, len(kernel)//2), 'constant')
    for i in range(array.size-((len(kernel)//2) *2)):
        end_array[i] = np.sum(array[i:i+len(kernel)] * kernel)
    end_array = end_array[1:-1]
    return end_array

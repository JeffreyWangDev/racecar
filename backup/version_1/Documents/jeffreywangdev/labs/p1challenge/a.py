import sys
import cv2 as cv
import numpy as np
import math

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

MIN_CONTOUR_AREA = 10

global queue
queue = []

CROP_FLOOR = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))

coneColor = None
Purple = ((120, 120, 122), (179, 255, 255))
Orange = ((166, 254 - 20, 254 - 10), (24, 255, 255))
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

# Add any global variables here
########################################################################################
# States
########################################################################################
class State(IntEnum):
    Search = 0
    OrangeCurve = 1
    PurpleCurve = 2

cur_state = State.Search
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global cur_state
    cur_state = State.OrangeCurve
    print(">> Phase 1 Challenge: Cone Slaloming")


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        print("image got")
        # Find all of the Orange and Purple contours
        OrangeContours = rc_utils.find_contours(image, Orange[0], Orange[1])
        PurpleContours = rc_utils.find_contours(image, Purple[0], Purple[1])

        # Select the largest Orange and Purple contour
        OrangeContour = rc_utils.get_largest_contour(OrangeContours, MIN_CONTOUR_AREA)
        PurpleContour = rc_utils.get_largest_contour(PurpleContours, MIN_CONTOUR_AREA)

        if OrangeContour is None and PurpleContour is None:
            contour = None
        elif OrangeContour is None:
            contour = PurpleContour
            coneColor = "Purple"
        elif PurpleContour is None:
            contour = OrangeContour
            coneColor = "Orange"
        else:
            if rc_utils.get_contour_area(OrangeContour) > rc_utils.get_contour_area(PurpleContour):
                contour = OrangeContour
                coneColor = "Orange"
            else:
                contour = PurpleContour
                coneColor = "Purple"
        print("coneColor")
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


def PurpleCurve(contour_center, contour_area):
    global queue

    if contour_area < 2500 and contour_area > 200:
        rc.drive.set_speed_angle(0.2, -0.2)
    elif contour_center is None or contour_area < 200:
        rc.drive.set_speed_angle(0.2, 0.2)
    else:
        TURN_ANGLE = ((contour_center[1] - 590) / 320)
        if TURN_ANGLE > 1:
            TURN_ANGLE = 1
        elif TURN_ANGLE < -1:
            TURN_ANGLE = -1
        # Gradually adjust the angle to prevent oscillation
        TURN_ANGLE = 0.8 * TURN_ANGLE + 0.2 * queue[-1] if queue else TURN_ANGLE
        queue.append(TURN_ANGLE)
        rc.drive.set_speed_angle(0.2, TURN_ANGLE)


def OrangeCurve(contour_center, contour_area):
    global queue

    if contour_area < 2500:
        rc.drive.set_speed_angle(0.2, 0.2)
    elif contour_center is None:
        rc.drive.set_speed_angle(0.2, -0.2)
    else:
        TURN_ANGLE = ((contour_center[1] - 50) / 320)
        if TURN_ANGLE > 1:
            TURN_ANGLE = 1
        elif TURN_ANGLE < -1:
            TURN_ANGLE = -1
        # Gradually adjust the angle to prevent oscillation
        TURN_ANGLE = 0.8 * TURN_ANGLE + 0.2 * queue[-1] if queue else TURN_ANGLE
        queue.append(TURN_ANGLE)
        rc.drive.set_speed_angle(0.2, TURN_ANGLE)


def update():
    global cur_state
    global coneColor
    global contour_center
    global contour_area

    update_contour()

    if coneColor == "Orange":
        cur_state = State.OrangeCurve
    elif coneColor == "Purple":
        cur_state = State.PurpleCurve

    if cur_state == State.OrangeCurve:
        OrangeCurve(contour_center, contour_area)
    elif cur_state == State.PurpleCurve:
        PurpleCurve(contour_center, contour_area)
    print(coneColor)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update(). By default, update_slow() is run once per second
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

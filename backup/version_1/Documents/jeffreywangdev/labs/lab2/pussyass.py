"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
queue = []
# Queue [time in seconds [speed,wheel]]
def make_zero():
    global queue
    queue.clear()
    rc.drive.stop()
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()
    rc.drive.set_speed_angle(0, 0)
    make_zero()
    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
    )

def update():
    global queue
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO (warmup): Implement acceleration and steering
    speed=0.2
    if rc.controller.was_pressed(rc.controller.Button.A):
        queue.append([2.48,[speed,1]])
    if rc.controller.was_pressed(rc.controller.Button.B):
        print("Driving in a square...")
        queue.append([.6,[speed,0]])
        queue.append([.3,[speed,1]])
        queue.append([.3,[speed,0]])
        queue.append([.3,[speed,-1]])
        queue.append([.3,[speed,0]])
        queue.append([.3,[speed,-1]])
        queue.append([.3,[speed,0]])
        queue.append([.3,[speed,1]])
        queue.append([.3,[speed,0]])


        queue.append([0.6,[speed,1]])
        queue.append([0.5,[speed-0.05,0]])
        queue.append([0.6,[speed-0.008,-1]])
        queue.append([0.5,[speed-0.07,0]])
        queue.append([0.5,[speed,-1]])
        queue.append([0.5,[speed-0.1,0]])
        queue.append([0.5,[speed,1]])
        queue.append([0.5,[speed-0.1,0]])

        queue.append([0.6,[speed,1]])
        queue.append([0.5,[speed-0.05,0]])
        queue.append([0.6,[speed-0.008,-1]])
        queue.append([0.5,[speed-0.07,0]])
        queue.append([0.5,[speed,-1]])
        queue.append([0.5,[speed-0.1,0]])
        queue.append([0.5,[speed,1]])
        queue.append([0.5,[speed-0.1,0]])


    if rc.controller.was_pressed(rc.controller.Button.X):
        print("Figure 8")
        queue.append([2,[speed,1]])
        queue.append([2,[speed,-1]])
        queue.append([0.5,[speed,1]])
        # TODO (main challenge): Drive in a square when the B button is pressed
    
    
    if rc.controller.was_pressed(rc.controller.Button.Y):
        print("Triangle")
        queue.append([0.5,[speed,0]])
        queue.append([0.6,[speed,1]])
        queue.append([0.5,[speed,0]])
        queue.append([0.6,[speed,1]])
        queue.append([0.5,[speed,0]])
        queue.append([0.6,[speed,1]])




    if queue:
        if queue[0][0]<=0:
            queue.pop(0)
            rc.drive.stop()
        else:
            queue[0][0]-=rc.get_delta_time()
            rc.drive.set_speed_angle(queue[0][1][0],queue[0][1][1])
        
    # TODO (main challenge): Drive in a figure eight when the X button is pressed

    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()

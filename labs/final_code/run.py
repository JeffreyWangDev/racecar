from Control import Control
from Camera import Camera
from States import States
from imports import *

obstacles = {
                1:States.Speed,
                2:States.Wall,
                3:States.Wall,
                4:States.Wall,
                5:States.Cone,
                6:States.Wall,#ramp
                7:States.Wall,9:States.Wall,10:States.Wall,11:States.Wall,12:States.Wall,13:States.Wall,14:States.Wall,15:States.Wall,16:States.Wall,
                8:States.Wall
            }
control=Control()
camera=Camera()
rc = racecar_core.create_racecar()
state = States.Wall
prev_state = States.Line
id = 1
angle = 0
speed = 0
def start():
    rc.set_update_slow_time(0.5)
    control.reset()
clock = 1
def update_all():
    global id,speed,angle,state,prev_state,clock
    dt = rc.get_delta_time()
    
    image = rc.camera.get_color_image()
    cheek_id = camera.aruco.update(image)
    if cheek_id is None:
        pass
    else:
        if state != obstacles[id]:
            prev_state = state
            state = obstacles[id]
            id = cheek_id
    lidar = rc.lidar.get_samples()
    if state == States.Speed:
        speed = 0.3
        angle = 0
    if state == States.Line:
        camera.line.update(image)
        angle = control.line.update(dt,camera.line.contour_center)
        if id == 4 or id == 8:
            speed = States.Line.fast_speed
        else:
            speed = States.Line.slow_speed
        print(angle)
        return
    if state == States.Cone:
        contour_center,contour_area,cone_color = camera.cone.update(image)
        control.cone.update(camera.cone.coneColor,camera.cone.contour_center,camera.cone.contour_area)
        angle = control.cone.angle
        speed = control.cone.speed
        return
    if state == States.Wall:
        if id == 6:
            speed,angle = control.wall.update(lidar,280,70)
        else:
            speed,angle = control.wall.update(lidar,70,70)
        angle = angle
        if speed == 1:
            speed = States.Wall.turn_speed
        else:
            speed = States.Wall.straight_speed
        if id ==4:
            speed = States.Wall.fast
        return

def update():
    a = 1
    if rc.controller.is_down(rc.controller.Button.B):
        a = 0.5
    if rc.controller.is_down(rc.controller.Button.Y):
        rc.drive.set_speed_angle(a,0)
        return
    if rc.controller.is_down(rc.controller.Button.A):
        rc.drive.set_speed_angle(a,angle)
        return
    
    update_all()
    rc.drive.set_speed_angle(speed,angle)

def slow_update():
    print("State: ",state.name, id)
    print(f"Angle: {angle}    Speed: {speed}")

rc.set_start_update(start, update,slow_update)
rc.go()
from Control import Control
from Camera import Camera
from imports import *
control=Control()
camera=Camera()
rc = racecar_core.create_racecar()
def start():
    rc.set_update_slow_time(0.5)

def update():
    angle = 0
    dt = rc.get_delta_time()
    lidar = rc.lidar.get_samples()
    angle = control.wall.update(lidar,dt)[1]
    control.speed.speed_speed = 12
    speed = 0.15    
    rc.drive.set_speed_angle(speed,angle)

def slow_update():
    print("Contour center: ",camera.line.contour_center)
    print("Color detected: ",camera.line.color)


rc.set_start_update(start, update,slow_update)
rc.go()

from Control import Control
from Camera import Camera
from imports import *
class Car:
    def __init__(self) -> None:
        self.control=Control()
        self.camera=Camera()
        self.rc = racecar_core.create_racecar()
    def start(self):
        self.rc.set_update_slow_time(0.5)

    def update(self):
        angle = 0
        dt = self.rc.get_delta_time()
        x_pos = self.camera.line.update(self.rc.camera.get_color_image())
        if x_pos:
            angle = self.control.wall.update(dt,x_pos) 
        self.control.speed.speed_speed = 12
        speed = 0.15    
        self.rc.drive.set_speed_angle(speed,angle)

    def slow_update(self):
        print("Contour center: ",self.camera.line.contour_center)
        print("Color detected: ",self.camera.line.color)
    def start(self):
        self.rc.set_start_update(self.start, self.update,self.slow_update)
        self.rc.go()
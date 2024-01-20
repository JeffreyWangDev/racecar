from Control import Control
from Camera import Camera
from States import States
from imports import *
class Car:
    def __init__(self) -> None:
        self.obstacles = {1:States.Wall,
                          2:States.Line,
                          3:States.Wall,
                          4:States.Wall,
                          5:States.Cone,
                          6:States.Lane,
                          7:States.Wall,9:States.Wall,10:States.Wall,11:States.Wall,12:States.Wall,13:States.Wall,14:States.Wall,15:States.Wall,16:States.Wall,
                          8:States.Wall}
        self.control=Control()
        self.camera=Camera()
        self.rc = racecar_core.create_racecar()
        self.state = States.Wall
        self.angle = 0
        self.speed = 0
    def start(self):
        self.rc.set_update_slow_time(0.5)

    def update_all(self):
        image = self.rc.camera.get_color_image()
        dt = self.rc.get_delta_time()
        id = self.camera.aruco.update(image)
        if id is None:
            return
        self.state = self.obstacles[id]
        
        if self.state == States.Line:
            self.camera.line.update(image)
            angle = self.control.line.update(dt,self.camera.line.contour_center)
            self.angle = angle
            if id == 1 or id == 4 or id == 8:
                self.speed = States.Line.fast_speed
            else:
                self.speed = States.Line.slow_speed
            return
        if self.state == States.Cone:
            contour_center,contour_area,cone_color = self.camera.cone.update(image)
            self.control.cone.update(self.camera.cone.coneColor,self.camera.cone.contour_center,self.camera.cone.contour_area)
            self.angle = self.control.cone.angle
            self.speed = self.control.cone.speed
            return
        if self.state == States.Wall:
            lidar = self.rc.lidar.get_samples()
            speed,angle = self.control.wall.update(lidar,dt)
            self.angle = angle
            if speed == 1:
                self.speed = States.Wall.turn_speed
            else:
                self.speed = States.Wall.straight_speed
            return
    
    def update(self):
        self.update_all()
        self.rc.drive.set_speed_angle(self.speed,self.angle)
    
    def slow_update(self):
        print("State: ",self.state.name)
        print(f"Angle: {self.angle}    Speed: {self.speed}")
        
    def start(self):
        self.rc.set_start_update(self.start, self.update,self.slow_update)
        self.rc.go()

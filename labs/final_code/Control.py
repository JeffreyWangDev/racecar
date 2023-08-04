from imports import *
from States import States
class Control:
    def __init__(self) -> None:
        self.speed = self.Speed()
        self.line = self.Line()
        self.wall = self.Wall()
        self.cone = self.Cone()
    class Speed:
        def __init__(self) -> None:
            self.speed_speed = 0.2                        # Set speed
            self.__speed_PID_P = 0.4                      # PID P term
            self.__speed_PID_I = 0.1                      # PID I term
            self.__speed_PID_D = 0.08                     # PID D term
            self.__speed_V0 = 0                           # Previous speed
            self.__speed_V1 = 0                           # Current speed
            self.__speed_power = 0                        # Current power level
            self.__speed_velocity_list=[0,0,0,0,0]        # List of speed changes
            self.__speed_power_list=[0 for i in range(10)]# List of average speed changes
            self.__speed_accumulated_error = 0.0          # PID addtive error for I
            self.__speed_last_error = 0.0                 # Previous PID error
        def update(self,dt:float,angular_v:tuple) -> float: 
            """
            Updates the speed PID control
            
            Args:
                dt (float): Delta time
                angular_v (tuple): Angular velocity
            Returns:
                float: Power level for the motor
            """
            self.__speed_V1 = angular_v[2]/dt                                                               # Change in speed
            self.__speed_v = self.__speed_V1-self.__speed_V0                                                # Current speed
            self.__speed_velocity_list.append(self.__speed_v)                                               # Add current speed to list
            self.__speed_velocity_list.pop(0)                                                               # Remove oldest speed from list
            self.__speed_average_v=average(self.__speed_velocity_list)                                      # Average speed
            self.__speed_power,self.__speed_accumulated_error,self.__speed_last_error = pid_control(        # PID control
                self.__speed_PID_P, self.__speed_PID_I, self.__speed_PID_D,                                 # PID constants
                self.speed_speed, self.__speed_average_v,                                                   # Set speed, current speed
                self.__speed_accumulated_error,                                                             # PID addtive error for I
                self.__speed_last_error, dt                                                                 # Previous PID error, delta time
                )                                                                   
            speed_power = clamp(self.__speed_power, -1, 1)                                                  # Clamp power
            self.__speed_power_list.append(speed_power)                                                     # Add current power to list
            self.__speed_power_list.pop(0)                                                                  # Remove oldest power from list
            self.speed_V0 = self.__speed_V1                                                                 # Set previous speed to current speed
            return average(self.__speed_power_list)                                                         # Return average power
    class Line:
        def __init__(self) -> None:
            self.__line_PID_P = 0.4                      # PID P term
            self.__line_PID_I = 0.1                      # PID I term
            self.__line_PID_D = 0.08                     # PID D term
            self.__line_accumulated_error = 0.0          # PID addtive error for I
            self.__line_last_error = 0.0                 # Previous PID error
            self.__line_set_point = 160
        def update(self,dt:float,x_pos:int) -> float:
            angle,self.__line_accumulated_error,self.__line_last_error = pid_control(self.__line_PID_P, self.__line_PID_I, self.__line_PID_D, self.__line_set_point, x_pos, self.__line_accumulated_error, self.__line_last_error, dt)
            angle = remap_range(angle, -320,320, 1, -1)
            angle= clamp(angle, -1, 1)
            return angle
    class Wall:
        def __init__(self) -> None:
            self.__wall_PID_P = 0.1                      # PID P term
            self.__wall_PID_I = 0.0                      # PID I term
            self.__wall_PID_D = 0.3                      # PID D term
            self.__wall_accumulated_error = 0.0          # PID addtive error for I
            self.__wall_last_error = 0.0                 # Previous PID error
            self.__wall_set_point = 160
            self.sharp_front_distance=140

            self.dset=50
        def update(self,lidar_scan,dt):
            rightdist= rc_utils.get_lidar_average_distance(lidar_scan,90, 20)
            frontdist = rc_utils.get_lidar_average_distance(lidar_scan,0, 40)
            if(frontdist<self.sharp_front_distance):
                shift_scan=np.roll(lidar_scan,len(lidar_scan)//4)
                maxangle=np.argmax(shift_scan[0:len(lidar_scan)//2]) #only look at front
                if (maxangle<(len(lidar_scan)//4)):
                    
                    return 1,-1
                else:   
                    return 1,1

            angle= 0.1 * (rightdist - 50)+0.3*(rightdist-self.__wall_last_error)
            self.__wall_last_error= rightdist
                #np.argmin(scan[0:359])/2-90
            angle/=50

            angle/=self.dset

            angle=clamp(angle,-1,1)

            return 0,angle
    class Cone:
        class State(IntEnum):
            Search = 0
            orangeCurve = 1
            purpleCurve = 2
        def __init__(self) -> None:

            self.speed = 0
            self.angle = 0
            self.cur_state: self.State = self.State.Search

        def purpleCurve(self,contour_center):
            maxa = 0.13
            speeda = States.Cone.fast_speed
            if contour_center is None:
                self.angle = 0.149
                self.speed = speeda*0.9
                return(speeda*0.9, 0.146)
            else:
                TURN_ANGLE = ((contour_center[1] - 800) / 320) - 0.08
                TURN_ANGLE = clamp(TURN_ANGLE, -maxa, maxa)
                
                if -maxa < TURN_ANGLE < maxa:
                    TURN_ANGLE = 0
                self.angle = TURN_ANGLE
                self.speed = speeda
                return(speeda,TURN_ANGLE)

        def orangeCurve(self,contour_center):
            maxa = 0.13
            speeda = States.Cone.fast_speed
            if contour_center is None:
                self.angle = -0.149
                self.speed = speeda*0.9 
                return(speeda*0.9, -0.146)
            else:
                TURN_ANGLE = ((contour_center[1] - 800) / 320) + 0.08
                TURN_ANGLE = clamp(TURN_ANGLE, -maxa, maxa)
                TURN_ANGLE = remap_range(TURN_ANGLE,1,-1,-1,1)
                if -maxa < TURN_ANGLE < maxa:
                    TURN_ANGLE = 0
                self.angle = TURN_ANGLE
                self.speed = speeda
                return(speeda,TURN_ANGLE)
        def update(self,coneColor,contour_center,contour_area) -> None:


            if coneColor == "orange":
                self.cur_state = Control.Cone.State.orangeCurve
            elif coneColor == "purple":
                self.cur_state = Control.Cone.State.purpleCurve
            
            if self.cur_state == Control.Cone.State.orangeCurve:
                self.orangeCurve(contour_center)
            elif self.cur_state == Control.Cone.State.purpleCurve:
                self.purpleCurve(contour_center)
            elif self.cur_state == Control.Cone.State.Search:
                self.angle = 0
                self.speed = States.Cone.fast_speed
                return(self.speed,self.angle)
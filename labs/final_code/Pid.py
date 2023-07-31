from imports import *

class Pid:
    def __init__(self) -> None:
        self.speed = self.Speed()
        self.line = self.Line()
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
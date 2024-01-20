import math

model_c = 0.6       # Model covariance 
sensor_c = 0.15     # Sensor covariance 
speed = 1.2         # Speed from IMU

gyro_data = [(0.25, [0.09, 0.00, 0.85]), (0.5, [0.09, 0.00, 0.85]), (0.75, [0.09, 0.00, 0.85]), (1.00, [0.09, 0.00, 0.85]), (1.25, [0.09, 0.00, 0.85]), (1.50, [0.09, 0.00, 0.85]), (1.75, [0.09, 0.00, 0.85]), (2.00, [0.09, 0.00, 0.85])]
colorcam_angle = [(0.25, 5.45), (0.50, 10.12), (0.75, 15.25), (1.00, 20.12), (1.25, 24.95), (1.50, 29.15), (1.75, 34.62), (2.00, 39.19)]

total_a = 0

for i in range(len(colorcam_angle)):
    Kg = 0.8     
    total_a =gyro_data[i][1][2]*gyro_data[i][0]*180/math.pi
    Xt = total_a + Kg*(colorcam_angle[i][1]-(total_a))    # Calculate current pos
    Eest = (1-Kg)*sensor_c                      # Calculate error
    print(gyro_data[i][0],Xt,Kg)
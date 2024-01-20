"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from nptyping import NDArray
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
distance=0
scan=[]
scan=np.array(scan)
lastd=0
total_lidar_pts=rc.lidar.get_num_samples()

# Add any global variables here

########################################################################################
# Functions
########################################################################################

def clamp(value: float, vmin: float, vmax: float) -> float:

    if value < vmin:
        return vmin
    elif value > vmax:
        return vmax
    else:
        return value
def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()
    #distance =rc_utils.get_lidar_average_distance(scan, 180)
    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")



# def show_lidar(
#     scan: NDArray[np.float32, np.float32],
#     radius: int = 128,
#     max_range: int = 400,
#     highlighted_samples: List[Tuple[int, int]] = []
# ) -> None:
#     """
#     Displays a visual representation of a LIDAR scan in Jupyter Notebook.
    
#     Args:
#         scan: The LIDAR scan to show.
#         radius: Half of the width and height (in pixels) of the generated image.
#         max_range: The farthest distance to show in the image in cm. Any sample past this range is not shown.
#         highlighted_samples: A list of samples in (angle, distance) format to show as a blue dot.
#     """    
#     # Create a square black image with the requested radius
#     image = np.zeros((2 * radius, 2 * radius, 3), np.uint8, "C")
#     num_samples: int = len(scan)

#     # TODO: Draw a green dot at the center of the image to denote the car
#     # Hint: Use rc_utils.draw_circle
#     CAR_DOT_RADIUS = 2


#     rc_utils.draw_circle(image, (radius, radius), (0, 255, 0), CAR_DOT_RADIUS)
        
#     # TODO: Draw a red pixel for each non-zero sample less than max_range
#     for i in range(len(scan)):
#         if scan[i] < max_range:
#             x= radius+int(math.cos(i/360*math.pi)*scan[i]/3)
#             y= radius+int(math.sin(i/360*math.pi)*scan[i]/3)
#             rc_utils.draw_circle(image, (x, y), (0, 0, 255), 2)
#     # TODO: Draw a light blue dot for each point in highlighted_samples
#     # Hint: Use rc_utils.draw_circle
#     HIGHLIGHT_DOT_RADIUS = 2
#     for i in highlighted_samples:
#         x = int(i[0]/3)
#         y = int(i[1]/3)
#         rc_utils.draw_circle(image, (x, y), (230, 216, 173), 2)

#     # Show the image with Matplotlib
#     plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
#     plt.show()

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global angle
    global scan
    global sharp
    global lastd
    global total_lidar_pts
    # TODO: Follow the wall to the right of the car without hitting anything.
    #scan=[]
    scan= rc.lidar.get_samples()
 #   scan=np.array(scan)
    # TBD=0
    #print(distance)
    # if(rc_utils.get_lidar_average_distance(scan, 180,20) == distance):
    # if(np.argmin(scan))

    #     angle= 0
    # else:
    ### ONE SIDE OF WALL
    Kp=0.1
    Kd=0.3
    dset=50
    sharp_front_distance=140
    straight_speed=0.15
    turn_speed =0.155
    #remapvalue=150- dset
    remapvalue= dset
    rightdist= rc_utils.get_lidar_average_distance(scan,90, 20)
    frontdist = rc_utils.get_lidar_average_distance(scan,0, 40)
    print('right distance',rightdist)
    print('front distance',frontdist)
    sharp=False
    if(frontdist<sharp_front_distance):
        print("hello")
        print("min",(np.argmin(scan)))
        # min= np.argmin(scan)
        # if(160>min and min<200):
        #     print("hello1")
        #     sharp = True
        #     rc.drive.set_speed_angle(0.1,-1)
        # if(min>520  and min<560):
        #     rc.drive.set_speed_angle(0.1,1)
        #     sharp = True
        shift_scan=np.roll(scan,total_lidar_pts//4)
        maxangle=np.argmax(shift_scan[0:total_lidar_pts//2]) #only look at front
        print("max",(maxangle))
        if (maxangle<(rc.lidar.get_num_samples()//4)):
            print("hello1")

            rc.drive.set_speed_angle(turn_speed,-1)  
        else:
            rc.drive.set_speed_angle(turn_speed,1)  
        sharp = True                     
    # print("angle b4 clamp",np.argmin(scan[0:359]))
    # angle/=90
# if front too close, turn

    #angle=np.argmax(np.concatenate((scan[540:719],scan[0:179])))/2 -90
    if(not sharp):
        angle= Kp * (rightdist - dset)+Kd*(rightdist-lastd)
        lastd= rightdist
        #np.argmin(scan[0:359])/2-90
        angle/=remapvalue
        print('old angle',angle)
#        angle/=90


        angle=clamp(angle,-1,1)
        print('new angle:',angle)
    # distance = rc_utils.get_lidar_average_distance(scan,180,20)
    

        rc.drive.set_speed_angle(straight_speed,angle)


    # p= scan
    # #k= [-1,-2,-3,5,-3,-2,-1]
    # k=[-3,-3,5,-3,-3]
    # d=[]

    # stdev= np.std(p)
    # clusters={}
    # cluster_index=np.zeros(len(scan))
    # #cluster=[i+j]
    # cluster_num=0
    # #Ci=[]

    # m=len(k)
    # print(len(p))
    # for l in range(1,len(scan)):
    #     d.append(p[l]-p[l-1])
    # print(len(d))
    # for i in range(1,len(p)-1):
    #     total=0
    #     for j in range(-(m-1)//2 , (m-1)//2):
    #     #  print(i+j)
    #         if not ((i+j)<0 or (i+j)>len(scan)-2):
    #             total=total+ d[i+j]*k[(j+(int)((m+1)/2))]
        

        
    #     if total>stdev:
    #         clusters[total]=[p[i]]
    #         C=total
    #     else:
    #         clusters[C]+=[scan[i]]
    #     print(clusters)

        



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
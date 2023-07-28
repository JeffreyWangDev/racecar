import cv2
import numpy as np
import glob

img_array = []
for filename in glob.glob('*.png'):
   img = cv2.imread(filename)
   height, width, layers = img.shape
   size = (width,height)
   img_array.append(img)
   print(1)
out = cv2.VideoWriter('video.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

for i in range(len(img_array)):
   out.write(img_array[i])
out.release()
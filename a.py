import cv2

dict = {"[[0, 0, 0, 1, 1, 1], [1, 0, 0, 0, 1, 1], [1, 1, 0, 1, 1, 1], [0, 1, 1, 0, 0, 0], [0, 0, 1, 0, 1, 0], [1, 0, 0, 1, 1, 0]]":1,"[[1, 1, 0, 0, 1, 0], [0, 1, 0, 0, 0, 1], [1, 0, 1, 1, 0, 0], [1, 1, 0, 0, 0, 0], [0, 1, 1, 0, 1, 0], [0, 1, 1, 1, 1, 0]]":2,"[[0, 1, 0, 1, 1, 1], [1, 1, 0, 0, 1, 0], [0, 0, 1, 1, 0, 0], [1, 1, 1, 0, 1, 1], [0, 1, 1, 0, 1, 1], [1, 1, 1, 1, 1, 1]]":3,"[[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 1, 1, 1, 1, 0]]":4}

def find_ar(img):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    split = []

    for i in range(1,7):
        a = []
        for j in range(1,7):
            imga = gray[int((img.shape[0]/8)*i):int((img.shape[0]/8)*(i+1)),int((img.shape[1]/8)*j):int((img.shape[1]/8)*(j+1))]
            if str(list(imga)).count("255") > str(list(imga)).count("0"):
                a.append(1)
            else:
                a.append(0)
        split.append(a)
    return(dict[str(split)])
video = cv2.VideoCapture('video.mp4')
go = True
while go:
    go, image = video.read()
    if image is not None:
        print(find_ar(image))
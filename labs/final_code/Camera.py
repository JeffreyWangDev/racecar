from imports import *

class Camera:
    def __init__(self) -> None:
        self.aruco = self.Aruco()
        self.line = self.Line()
        self.cone = self.Cone()
    class Aruco:
        def __init__(self) -> None:
            self.marker = None
            self.id = -1
            self.center = (0,0)
        def update(self,image) -> int:
            try:
                maker = rc_utils.get_ar_markers(image)
                if maker is not None:
                    self.marker = maker[0]
                    self.id = self.marker.get_id()
                    corners = self.marker.get_corners()
                    self.center = ((corners[0][0]-corners[-1][0])/2,(corners[0][1]-corners[-1][1])/2)
                    return self.id
                return None
            except:
                pass
    
    class Line:
        def __init__(self) -> None:
            self.contour_center = 160
            self.color = None

        @staticmethod
        def find_contours(image,color=None) -> float:
            
            if color!=None:
                return rc_utils.find_contours(image, color.lower_value, color.upper_value)
            return {
                "blue": rc_utils.find_contours(image, Colors.Lines.Blue.lower_value, Colors.Lines.Blue.upper_value),
                "green": rc_utils.find_contours(image, Colors.Lines.Green.lower_value, Colors.Lines.Green.upper_value),
                "red": rc_utils.find_contours(image, Colors.Lines.Red.lower_value, Colors.Lines.Red.upper_value),
                "yellow": rc_utils.find_contours(image, Colors.Lines.Yellow.lower_value, Colors.Lines.Yellow.upper_value),
            }
        @staticmethod
        def find_largest_contour(contour,min = 40) -> Any:            
            return rc_utils.get_largest_contour(contour,min)
        @staticmethod
        def find_center_contour(contour) -> tuple:
            return rc_utils.get_contour_center(contour)
        @staticmethod
        def preprocess_image(image) -> Any:
            image_copy = image[50:len(image)]
            return image_copy
        def update(self,image,color_priority = [Colors.Lines.Blue]) -> int:
            image = self.preprocess_image(image)
            for i in color_priority:
                contours = self.find_contours(image,i)
                if contours is not None:
                    print(i.name)
                    largest = self.find_largest_contour(contours)
                    if largest is not None:
                        center = self.find_center_contour(largest)
                        self.contour_center = center[1]
                        self.color = i.name
                        return center[0]
    class Cone:
        def __init__(self) -> None:
            self.__min_countour_area = 500
            self.coneColor = None
            self.contour_center = None 
            self.contour_area = 0
            self.CROP_FLOOR = ((100,0), (240, 320))
        def update(self,image):
            if image is None:
                self.contour_center = None
                self.contour_area = 0
            else:

                image = rc_utils.crop(image, self.CROP_FLOOR[0], self.CROP_FLOOR[1])

                # Find all of the orange and purple contours
                orangeContours = Camera.Line.find_contours(image,Colors.Cones.Orange)      
                purpleContours = Camera.Line.find_contours(image,Colors.Cones.Purple)

                # Select the largest orange and purple contour
                
                orangeContour = Camera.Line.find_largest_contour(orangeContours,self.__min_countour_area)
                purpleContour = Camera.Line.find_largest_contour(purpleContours,self.__min_countour_area)
                if orangeContour is None and purpleContour is None:
                    contour = None
                elif orangeContour is None:
                    contour = purpleContour
                    self.coneColor = "purple"
                elif purpleContour is None:
                    contour = orangeContour
                    self.coneColor = "orange"
                else:
                    if rc_utils.get_contour_area(orangeContour) > rc_utils.get_contour_area(purpleContour):
                        contour = orangeContour
                        self.coneColor = "orange"
                    else:
                        contour = purpleContour
                        self.coneColor = "purple"
                if contour is not None:
                    # Calculate contour information
                    self.contour_center = rc_utils.get_contour_center(contour)
                    self.contour_area = rc_utils.get_contour_area(contour)
                else:
                    self.contour_center = None
                    self.contour_area = 0
            return self.contour_center,self.contour_area,self.coneColor
    class Lane:
        
        def __init__(self) -> None:
            self.greatestcontour = None
            self.secondgreatest = None
        
        def get_largest_and_second_contour(contours, MIN_CONTOUR_AREA):
            contours = list(contours)
            count = 0
            for contour in contours:
                if contour is not None:
                    if cv.contourArea(contour) > MIN_CONTOUR_AREA:
                        count += 1
                else:
                    count += 1
            if count == 0:
                return None, None
            index = 0
            greatest = 0
            greatestcontour = None
            for i in range(len(contours)):
                if cv.contourArea(contours[i]) > greatest:
                    greatest = cv.contourArea(contours[i])
                    index = i
                    greatestcontour = contours[i]
            contours.pop(index)
            
            greatest = 0
            secondgreatest = None
            for i in range(len(contours)):
                if cv.contourArea(contours[i]) > greatest:
                    greatest = cv.contourArea(contours[i])
                    secondgreatest = contours[i]
            return greatestcontour, secondgreatest

        #This iterates through the priority list until it finds contours that are greater than the min contour area and gets the contour center of the largest and second              
        def update_contour_two_line(image ):
            MIN_CONTOUR_AREA = 50
            CROP_FLOOR = ((180,0), (240, 320))
            image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
            largestcontour = None
            secondcontour = None
            largestcontour_center = (0, 0)
            secondcontour_center = (0, 0)
            generalcontour_center = (0, 0) #This is the center if only one contour is seen on the screen
            hsvTarget=((Colors.Lines.Blue.lower_value,Colors.Lines.Blue.upper_value))
            prioritylist = [hsvTarget]

            for col in prioritylist:
                contours = rc_utils.find_contours(image, col[0], col[1])
                largestcontour, secondcontour = Camera.Lane.get_largest_and_second_contour(contours, MIN_CONTOUR_AREA)
                if (largestcontour is not None) and (secondcontour is not None):
                    break
            if largestcontour is not None and secondcontour is not None:
                if largestcontour is not None:
                    largestcontour_center = rc_utils.get_contour_center(largestcontour)
                if secondcontour is not None:
                    secondcontour_center = rc_utils.get_contour_center(secondcontour)

            else:
                if largestcontour is not None:
                    generalcontour_center = rc_utils.get_contour_center(largestcontour)
                if secondcontour is not None:
                    generalcontour_center = rc_utils.get_contour_center(secondcontour)


            return largestcontour_center, secondcontour_center, generalcontour_center
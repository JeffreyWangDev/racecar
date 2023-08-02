from imports import *

class Camera:
    def __init__(self) -> None:
        self.aruco = self.Aruco()
        self.line = self.Line()
        self.image = None

    class Aruco:
        def __init__(self) -> None:
            self.marker = None
            self.id = -1
            self.center = (0,0)
        def update(self,image) -> int:
            maker = rc_utils.get_ar_markers(image)
            if maker is not None:
                self.marker = maker[0]
                self.id = self.marker.get_id()
                corners = self.marker.get_corners()
                self.center = ((corners[0][0]-corners[-1][0])/2,(corners[0][1]-corners[-1][1])/2)
                return self.id
            return None
    
    class Line:
        def __init__(self) -> None:
            self.contour_center = 160
            self.color = None

        @staticmethod
        def find_contours(image,color=None) -> float:
            
            if color!=None:
                return rc_utils.get_largest_contour(rc_utils.find_contours(image, color.lower_value, color.upper_value))
            return {
                "blue": rc_utils.find_contours(image, Colors.Lines.Blue.lower_value, Colors.Lines.Blue.upper_value),
                "green": rc_utils.find_contours(image, Colors.Lines.Green.lower_value, Colors.Lines.Green.upper_value),
                "red": rc_utils.find_contours(image, Colors.Lines.Red.lower_value, Colors.Lines.Red.upper_value),
                "yellow": rc_utils.find_contours(image, Colors.Lines.Yellow.lower_value, Colors.Lines.Yellow.upper_value),
            }
        @staticmethod
        def find_largest_contour(contour) -> Any:            
            return rc_utils.get_largest_contour(contour)
        @staticmethod
        def find_center_contour(contour) -> tuple:
            return rc_utils.get_contour_center(contour) 
        @staticmethod
        def preprocess_image(image) -> Any:
            image_copy = image[100:len(image)]
            return image_copy
        def update(self,image,color_priority = [Colors.Lines.Blue,Colors.Lines.Green,Colors.Lines.Red,Colors.Lines.Yellow]) -> int:
            for i in color_priority:
                contours = self.find_contours(image,i)
                if contours is not None:
                    largest = self.find_largest_contour(contours)
                    if largest is not None:
                        center = self.find_center_contour(largest)
                        self.contour_center = center[0]
                        self.color = i.name
                        return center[0]
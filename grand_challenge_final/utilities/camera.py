import cv2
import numpy as np
import math


class Camera:
    def __init__(self) -> None:
        self.focal_length = 320/math.tan(0.5 * math.radians(75))

    def debug(self, thresholded_image, blurred_image, edge_isolated_image, processed_image):
        return np.vstack((np.hstack((cv2.cvtColor(thresholded_image, cv2.COLOR_GRAY2BGR), cv2.cvtColor(blurred_image, cv2.COLOR_GRAY2BGR))), np.hstack((cv2.cvtColor(edge_isolated_image, cv2.COLOR_GRAY2BGR), processed_image))))

    def calculate_angle_from_center(self,x):
        return np.degrees(math.atan((x - 320)/self.focal_length))

    def resize(self, img, resolution=(640, 480)):
        return cv2.resize(img, resolution)
    
    def blur(self, img, kernel_size=9):
        return cv2.medianBlur(img, kernel_size)

    def color_filter(self, image):
        image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask0 = cv2.inRange(image_HSV, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(image_HSV, lower_red, upper_red)

        # join my masks
        mask = mask0+mask1
        return mask

    def detect_edge(self, img):
        edge_detected_image = cv2.Canny(img, 75, 200)
        return edge_detected_image

    def detect_contours(self, img, start_image):
        contours, _= cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # img1 = cv2.imread("./sample.png")
        # gray_img = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        # binary_image = cv2.threshold(gray_img, 1, 255, cv2.THRESH_BINARY)[1]
        # contours1, _= cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # temp = 1000
        
        # for contour in contours:
        #     if(temp > cv2.matchShapes(contour, contours1[0], 1, 0)):
        #         temp = cv2.matchShapes(contour, contours1[0], 1, 0)
        #         final_contour = contour
      
        # if(temp > 0.2):
        #     radius = 0
        # else:
        #     centers, radius = cv2.minEnclosingCircle(final_contour)

        radius = 0
        for contour in contours:
            approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
            area = cv2.contourArea(contour)
            # print(f"area {area}, shape {len(approx)}")
            if ((len(approx) > 6) & (area > 1000) ):
                # print(len(approx),area)
                centers, radius = cv2.minEnclosingCircle(contour)
                
        if(radius != 0):
            center = (int(centers[0]),int(centers[1]))
            start_image = cv2.circle(start_image, center, int(radius), (255,0,0), 2)
            start_image = cv2.circle(start_image, center, 2, (0,0,255), 2)
            
            angle = self.calculate_angle_from_center(center[0])
            cv2.putText(start_image,f"Green Light Detected", (10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
            cv2.putText(start_image,f"Location: {center}", (10,40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
            cv2.putText(start_image,f"Angle: {angle}", (10,60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
            cv2.putText(start_image,f"Y: {center[1]}", (10,80), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
            # flag =True

        else:
            cv2.putText(start_image,f"Green Light Not Detected", (10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
            # flag =False
            angle = 100000

        return start_image, angle
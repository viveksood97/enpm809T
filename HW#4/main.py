'''
@Copyright 2022 Vivek Sood
@file hw3.py
@author Vivek Sood
@date 02/07/2022
 
@brief Homework 1(ENPM 809T): Moving Average
 
@section LICENSE
 
MIT License
Copyright (c) 2022 Vivek Sood
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''  
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
import math

colors = {
    "test_red": ((0,50,50), (15,255,255)),
    "red": ((155,20,0), (179, 255, 255)),
    "green": ((55, 105, 28), (83, 255, 255)),
    "blue": ((62, 38, 0), (115, 255, 255))
}

def color_filter(image, color):
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
    # image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # mask  = cv2.inRange(image_HSV, colors[color][0], colors[color][1])
    return mask

def detect_edge(img):
    edge_detected_image = cv2.Canny(img, 75, 200)
    return edge_detected_image


def resize(img):
    scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def detect_contours(img, start_image):
    contours, _= cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_list = []
    radius = 0
    for contour in contours:
        approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
        # print(f"area {area}, shape {len(approx)}")
        if ((len(approx) > 6) & (area > 1000) ):
            # print(len(approx),area)
            contour_list.append(contour)
            centers, radius = cv2.minEnclosingCircle(contour)
            

    if(radius != 0):
        center = (int(centers[0]),int(centers[1]))
        start_image = cv2.circle(start_image, center, int(radius), (255,0,0), 2)
        start_image = cv2.circle(start_image, center, 2, (0,0,255), 2)
        focal_length = 320/math.tan(0.5 * math.radians(75))
        angle = math.degrees(math.atan((center[0] - 320)/focal_length))
        cv2.putText(start_image,f"Green Light Detected", (10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
        cv2.putText(start_image,f"Location: {center}", (10,40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
        cv2.putText(start_image,f"Angle: {angle}", (10,60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
        cv2.putText(start_image,f"Y: {center[1]}", (10,80), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
        flag =True

    else:
        cv2.putText(start_image,f"Green Light Not Detected", (10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,255,255), thickness=1)
        flag =False

    return start_image, flag

def main():
    frame_count = 0
    detection_count = 0
    np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})
    
    
    camera = cv2.VideoCapture(0)
    

    codec = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('./hw3.avi',codec, 15.0, (640,480))
    frame_time = []
    frame_count = 0
    while True:
        frame_count += 1
        start_time = time.time()
        ret, img = camera.read()
        	
        if not ret:
            break

        img = cv2.resize(img, (640, 480))
        
        #resized = resize(img)
        green_mask = color_filter(img, "red")#HSV Range = 55,105,50 83,255,255
        # kernel = np.ones((3,3), np.uint8)
 
        # The first parameter is the original image,
        # kernel is the matrix with which image is
        # convolved and third parameter is the number
        # of iterations, which will determine how much
        # you want to erode/dilate a given image.
        # img_erosion = cv2.erode(green_mask, kernel, iterations=1)
        # img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)
        median_blur= cv2.medianBlur(green_mask, 9)
        edge = detect_edge(median_blur)
        final, flag = detect_contours(edge,img)
        if(flag):
            detection_count += 1


        cv2.imshow('image', final)
        # out.write(final)
        # print(frame_count, detection_count)
        end_time = time.time()
        if(frame_count < 101):
            frame_time.append((end_time-start_time)*1000)
        frame_count += 1
        # time.sleep(5)
        key = cv2.waitKey(1) & 0xFF
        if(key == ord("q")):
            break
    camera.release()
    # out.release()
    cv2.destroyAllWindows()
    # frame_time.pop(0)
    # xPlot = [x for x in range(len(frame_time))]
    # plt.figure()
  
    # plt.plot(xPlot, frame_time, c='blue', label='Raw Data')
    
    # plt.title(f'Stop Light Detection Performance(without optimization)')
    # plt.xlabel('Frame')
    # plt.ylabel('Processing Time[msec]')
    # plt.legend()
    # plt.show()



if __name__ == '__main__':
    main()

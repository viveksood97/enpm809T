import RPi.GPIO as gpio
import time
import cv2
import numpy as np
from utilities.initialize import Initialize 
from utilities.motors import Motors
from utilities.encoders import Encoders
from utilities.camera import Camera
import serial

#Change
def ticks_to_travel(dist):
    return (dist/(65*np.pi))*40

def ticks_to_distance(ticks):
    return (ticks*(65*np.pi))/40

def localization(append_ticks, angle, trajectory_x, trajectory_y):
    if(angle < 0):
        angle = 360 + angle
    print(ticks_to_distance(append_ticks)*np.cos(np.deg2rad(angle)), ticks_to_distance(append_ticks)*np.sin(np.deg2rad(angle)))
    trajectory_x.append(trajectory_x[-1] + ticks_to_distance(append_ticks)*np.cos(np.deg2rad(angle)))
    trajectory_y.append(trajectory_y[-1] +ticks_to_distance(append_ticks)*np.sin(np.deg2rad(angle)))

def process_angle(line):
    line = line.rstrip().lstrip()
    line = str(line)
    line = line.strip("'")
    line = line.strip("b'")
    angle = float(line)
    
    
    if(angle > 180):
        current_angle = angle - 360
    else:
        current_angle = angle
    
    return current_angle

#ChangeEnd

def main():
   #Change
   trajectory_x = [0]
   trajectory_y = [0]
   append_ticks = 0
   initialize = Initialize()
   initialize.startup()
   motors = Motors(initialize.motor_pins, initialize.motor_pwm_objects)
   encoders = Encoders()

   ser = serial.Serial('/dev/ttyUSB0', 9600)
   count = 0
   #ChangeEnd

   np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

   # Video Recorder
   cap = cv2.VideoCapture(0)
   codec = cv2.VideoWriter_fourcc(*'XVID')
   out = cv2.VideoWriter('./hw3.avi',codec, 15.0, (640,480))

   camera = Camera()

   while(cap.isOpened()):
      ret, frame = cap.read()

      if not ret:
         break
      
      resized_image = camera.resize(frame)

      thresholded_image = camera.color_filter(resized_image)

      blurred_image = camera.blur(thresholded_image)

      edge_isolated_image = camera.detect_edge(blurred_image)

      processed_image, angle = camera.detect_contours(edge_isolated_image, resized_image)

      debug = camera.debug(thresholded_image, blurred_image, edge_isolated_image, processed_image)

      delta = 100000
      target_angle = angle
      prev_delta = 0
      duty_cycle = 30
      if(angle != 100000):
         print(target_angle)
         while(abs(delta) > 1):
               if(ser.in_waiting > 0):
                  count += 1
                  line = ser.readline()
                  
                  if(count > 10):
                     current_angle = process_angle(line)
               
               if(count > 10):
                  delta = target_angle - current_angle
                  if((360 - delta) < abs(delta)):
                     delta = delta - 360

                  diffrential = delta - prev_delta
                  pid = 1*(delta) + 0.01*(diffrential)
                  prev_delta = delta
                  # if(delta < 10):
                  #     print(delta)
                  # if(move == 180):
                  # print(target_angle, current_angle)
                  if(diffrential < 0.01):
                     break
                  if(delta < 0):
                     motors.move([0, max(min(43, duty_cycle + abs(pid)),0), 0, max(min(43, duty_cycle + abs(pid)),0)])
                     #motors.move([0, 35, 0, 35])
                           
                  else:
                     motors.move([max(min(43, duty_cycle + abs(pid)),0), 0, max(min(43, duty_cycle + abs(pid)),0), 0])

      cv2.imshow('Debugger',cv2.resize(debug,None,fx=0.8,fy=0.7))
      
      # cv2.imshow('Shape Detection', binary_image)
      # out.write(final)

      key = cv2.waitKey(1) & 0xFF
      if(key == ord("q")):
         break
   cap.release()
   # out.release()
   cv2.destroyAllWindows()





if __name__ == '__main__':
   start_time = time.time()
   main()
   end_time = time.time()
import RPi.GPIO as gpio
import cv2
import numpy as np
import time

from utilities.initialize import Initialize 
from utilities.motors import Motors
from utilities.servo import Servos
from test3 import KBHit

gpio.setmode(gpio.BOARD)
initialize = Initialize()
initialize.startup()
motors = Motors(initialize.motor_pins, initialize.motor_pwm_objects)
servos = Servos(initialize.servo_pins, initialize.servo_pwm_objects)
kb = KBHit()

print('Hit any key, or ESC to exit')
old_key = ""
gripper_open = True

def distance():
    trig = 16
    echo = 18

    gpio.output(trig, False)
    time.sleep(0.01)

    gpio.output(trig, True)
    time.sleep(0.00001)
    gpio.output(trig, False)


    while(gpio.input(echo)==0):
        pulse_start = time.time()
    
    while(gpio.input(echo)==1):
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration*17150
    distance = round(distance, 2)

    return distance

def key_input(event):
    print("Key: ", event)
    key_press = event
    tf = 1

    if key_press.lower() == 'w':
        motors.move([0, 60, 60, 0])
    elif key_press.lower() == 's':
        motors.move([60, 0, 0, 60])
    elif key_press.lower() == 'a':
        motors.move([0, 100, 0, 100])
    elif key_press.lower() == 'd':
        motors.move([100, 0, 100, 0])
    elif key_press.lower() == ' ':
        motors.move([0, 0, 0, 0])
    elif key_press.lower() == 'q':
        gripper_open = False
        servos.close_gripper()
    elif key_press.lower() == 'e':
        gripper_open = True
        servos.open_gripper()
    else:
        print("Closing....")
        servos.finish
        motors.finish()
        gpio.cleanup()



codec = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('./hw6.avi',codec, 15.0, (640,480))
frame_time = []
frame_count = 0
camera = cv2.VideoCapture(0)

while True:
    ret, img = camera.read()
    	
    if not ret:
        break
    distances = []
    for i in range(10):
        distances.append(distance())

    arr = np.array(distances)
    cv2.putText(img,f"Distance: {round(arr.mean()/100, 2)} m", (10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0,255,0), thickness=1)
    out.write(img)
    if kb.kbhit():
        c = kb.getch()
        
        if(old_key != c):
            motors.move([0, 0, 0, 0])
            key_input(c)
            if c == "p": # ESC
                break
    key = cv2.waitKey(1) & 0xFF
    if(key == ord("q")):
        break
camera.release()
out.release()
cv2.destroyAllWindows()
kb.set_normal_term()






    




# while True:
#     key_press = input("Direction please: ")
#     if key_press == 'p':
#         break
#     key_input(key_press)
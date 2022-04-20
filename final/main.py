from turtle import distance
import RPi.GPIO as gpio
import time
import numpy as np
from utilities.initialize import Initialize 
from utilities.motors import Motors
from utilities.encoders import Encoders
import matplotlib.pyplot as plt

start = time.time()
initialize = Initialize()
initialize.startup()
motors = Motors(initialize.motor_pins, initialize.motor_pwm_objects)
encoders = Encoders()
left = 0
right = 0

wheel_perimeter = 65*np.pi
ticks_to_travel = (2000/(wheel_perimeter))*20

delta_lis = []
time_lis = []
start = time.time()
start_2 = time.time()
sampleTicks = 5
delta_prev = 0
prev_left = 0
prev_right = 0
errorSum = 0
dutyCycle_l = 60
dutyCycle_r = 60
while(ticks_to_travel > left and ticks_to_travel > right):
    
    left, right = encoders.get_encoder_ticks()
    diff_l = left - prev_left
    diff_r = right - prev_right

    if(diff_l > diff_r):
        dutyCycle_l -= 5
    elif(diff_l < diff_r):
        dutyCycle_r -=5

    motors.move([0, max(min(100, dutyCycle_l),0), max(min(100, dutyCycle_r),0), 0])


motors.move([0, 0, 0, 0])
# plt.plot(time_lis, delta_lis)
# plt.savefig("p_kp_1")
# while True:
#     print(encoders.get_encoder_ticks())
#     time.sleep(1)

# while(time.time() - start < 2):
#     motors.move([0, 60, 60, 0])
motors.finish()
gpio.cleanup()



import RPi.GPIO as gpio
import time
import numpy as np
from utilities.initialize import Initialize 
from utilities.motors import Motors
from utilities.encoders import Encoders
import matplotlib.pyplot as plt
import serial

def ticks_to_travel(dist):
    return (dist/(65*np.pi))*40

travel = [ticks_to_travel(0), ticks_to_travel(500), ticks_to_travel(0), ticks_to_travel(500), ticks_to_travel(0), ticks_to_travel(500), ticks_to_travel(0), ticks_to_travel(500),  ticks_to_travel(0)]
# moves = ["backward", "forward"]
# moves = ["forward", "backward", "forward", "backward", "forward", "backward"]
# moves = ["forward", "backward"]
moves = [0, "forward", 90, "forward", 180, "forward", -90, "forward", 0]
# moves = [90, -90]

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

def main():
    initialize = Initialize()
    initialize.startup()
    motors = Motors(initialize.motor_pins, initialize.motor_pwm_objects)
    encoders = Encoders()

    ser = serial.Serial('/dev/ttyUSB0', 9600)
    count = 0
    #while(1):
    
    for index, move in enumerate(moves):
        left = 0
        right = 0
        prev_delta_encoder = 0
        encoders.encoders_reset()
        prev_delta = 0
        sum_delta = 0
        prev_angle_test = 0
        if(move == "forward" or move == "backward"):
            while(travel[index] > left + right):
                if(ser.in_waiting > 0):
                    count += 1
                    line = ser.readline()
                    
                    if(count > 10):
                        current_angle = process_angle(line)

                if(count > 10):
                    dutyCycle = 60
                    left, right = encoders.get_encoder_ticks()
                    
                    delta_encoder = left-right
                    
                    delta = moves[index - 1] - current_angle
                    if(360 - delta < abs(delta)):
                        delta = delta - 360

                    # pid = delta + 0.01*(delta-prev_delta)+  0.01*(sum_delta) + delta_encoder - prev_delta_encoder
                    pid = 11*delta + delta_encoder - prev_delta_encoder
                    prev_delta = delta
                    sum_delta += delta
                    prev_delta_encoder = delta_encoder
                    
                    if(move == "forward"):
                        if(index == 5 and abs(current_angle - prev_angle_test)  != 0):
                            print(delta, current_angle, moves[index - 1])
                            prev_angle_test = current_angle
                        motors.move([0, max(min(100, dutyCycle - pid),0), max(min(100, dutyCycle + pid),0), 0])
                    if(move == "backward"):
                        motors.move([max(min(100, dutyCycle + pid),0), 0, 0, max(min(100, dutyCycle - pid),0)])
        else:
            delta = 100000
            target_angle = move
            prev_delta = 0
            duty_cycle = 30

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
                    if(delta < 0):
                        motors.move([0, max(min(43, duty_cycle + abs(pid)),0), 0, max(min(43, duty_cycle + abs(pid)),0)])
                        #motors.move([0, 35, 0, 35])
                            
                    else:
                        motors.move([max(min(43, duty_cycle + abs(pid)),0), 0, max(min(43, duty_cycle + abs(pid)),0), 0])
                        # motors.move([35, 0, 35, 0])

                    
        motors.move([0, 0,0,0])

if __name__ == '__main__':
    start_time = time.time()
    main()
    end_time = time.time()
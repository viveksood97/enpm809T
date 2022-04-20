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

travel = [ticks_to_travel(1000), ticks_to_travel(1000), ticks_to_travel(1000), ticks_to_travel(1000), ticks_to_travel(1000), ticks_to_travel(1000)]
# moves = ["backward", "forward"]
# moves = ["forward", "backward", "forward", "backward", "forward", "backward"]
# moves = ["forward", "backward"]
moves = [-90, -0.1]

def main():
    initialize = Initialize()
    initialize.startup()
    motors = Motors(initialize.motor_pins, initialize.motor_pwm_objects)
    encoders = Encoders()

    ser = serial.Serial('/dev/ttyUSB0', 9600)
    count = 0
    #while(1):
    prev_delta = 0
    for index, move in enumerate(moves):
        left = 0
        right = 0
        prev_delta_encoder = 0
        encoders.encoders_reset()
        sum_delta = 0
        if(move == "forward" or move == "backward"):
            while(travel[index] > left + right):
                if(ser.in_waiting > 0):
                    count += 1
                    line = ser.readline()
                    
                    if(count > 10):
                        
                        line = line.rstrip().lstrip()

                        line = str(line)
                        line = line.strip("'")
                        line = line.strip("b'")
                        angle = float(line)
                        
                        
                        if(angle >= 180):
                            current_angle = angle - 360
                        else:
                            current_angle = angle
                        # current_angle = -1 * angle
                if(count > 10):
                    dutyCycle = 60
                    left, right = encoders.get_encoder_ticks()
                    
                    delta_encoder = left-right
                    delta = current_angle
                    pid = 10*delta + 0.01*(delta-prev_delta)+  0.01*(sum_delta) + delta_encoder - prev_delta_encoder
                    prev_delta = delta
                    sum_delta += delta
                    prev_delta_encoder = delta_encoder
                    
                    if(move == "forward"):
                        motors.move([0, max(min(100, dutyCycle + pid),0), max(min(100, dutyCycle - pid),0), 0])
                    if(move == "backward"):
                        motors.move([max(min(100, dutyCycle - pid),0), 0, 0, max(min(100, dutyCycle + pid),0)])
        else:
            target_angle = move
            if(target_angle < 0):
                to_move = [0, 60, 0, 60]
            else:
                to_move = [60, 0, 60, 0]
            flag = True
            
            while(flag):
                if(ser.in_waiting > 0):
                    count += 1
                    line = ser.readline()
                    
                    if(count > 10):
                        
                        line = line.rstrip().lstrip()

                        line = str(line)
                        line = line.strip("'")
                        line = line.strip("b'")
                        angle = float(line)
                        if(angle >= 180):
                            current_angle = angle - 360
                        else:
                            current_angle = angle
                        # current_angle = -1 * angle
                        
                
                if(count > 10):
                    delta = current_angle - target_angle
                    if(target_angle < 0):
                        print(delta, current_angle, target_angle)
                        if(delta > 0):
                            motors.move(to_move)
                        else:
                            flag = False
                    else:
                        if(delta < 0):
                            motors.move(to_move)
                        else:
                            flag = False

                    
        motors.move([0, 0,0,0])

if __name__ == '__main__':
    start_time = time.time()
    main()
    end_time = time.time()
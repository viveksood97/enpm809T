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

def ticks_to_distance(ticks):
    return (ticks*(65*np.pi))/40

def localization(append_ticks, angle, trajectory_x, trajectory_y):
    if(angle < 0):
        angle = 360 + angle
    print(ticks_to_distance(append_ticks)*np.cos(np.deg2rad(angle)), ticks_to_distance(append_ticks)*np.sin(np.deg2rad(angle)))
    trajectory_x.append(trajectory_x[-1] + ticks_to_distance(append_ticks)*np.cos(np.deg2rad(angle)))
    trajectory_y.append(trajectory_y[-1] +ticks_to_distance(append_ticks)*np.sin(np.deg2rad(angle)))


# travel = [ticks_to_travel(0), ticks_to_travel(500), ticks_to_travel(0), ticks_to_travel(500), ticks_to_travel(0), ticks_to_travel(500), ticks_to_travel(0), ticks_to_travel(500),  ticks_to_travel(0)]
# moves = [0, "forward", 90, "forward", 180, "forward", -90, "forward", 0]

travel = [ticks_to_travel(0), ticks_to_travel(510), ticks_to_travel(0), ticks_to_travel(505), ticks_to_travel(0), ticks_to_travel(490), ticks_to_travel(0), ticks_to_travel(525),  ticks_to_travel(0)]
moves = [0, "forward", 75, "forward", 194, "forward", -74, "forward", 15]


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
    trajectory_x = [0]
    trajectory_y = [0]
    append_ticks = 0
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
            
            append_ticks = left+right
            localization(append_ticks, moves[index - 1], trajectory_x, trajectory_y)
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
    # plt.plot(trajectory_x, trajectory_y)
    # plt.title("IMU + encoders")
    # plt.xlabel("X")
    # plt.ylabel("Y")
    # plt.grid()
    # plt.savefig("IMU")

if __name__ == '__main__':
    start_time = time.time()
    main()
    end_time = time.time()
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


def ticks_to_travel(distance):
    wheel_perimeter = 65*np.pi
    return (distance/(wheel_perimeter))*20



travel = [ticks_to_travel(1000), ticks_to_travel(234), ticks_to_travel(254.5), ticks_to_travel(1000), ticks_to_travel(200), ticks_to_travel(210)]
moves = ["forward",  "right", "left", "backward", "left", "right"]
# travel = [ticks_to_travel(235)]
# moves = ["right"]

time.sleep(5)


for index, move in enumerate(moves):
    left = 0
    right = 0
    sampleTicks = 5
    delta_prev = 0
    errorSum = 0
    encoders.encoders_reset()
    while(travel[index] > left and travel[index] > right):
        dutyCycle = 60
        left, right = encoders.get_encoder_ticks()
        if(sampleTicks > left or sampleTicks > right):
            delta = 11*(left - right) + 0.1*(left - right - delta_prev)

        # print(f"Delta: {delta}, dutyLeft: { max(min(99, dutyCycle + delta),0)}, DutyRight: { max(min(99, dutyCycle - delta),0)}")
            if(move == "forward"):
                motors.move([0, max(min(100, dutyCycle + delta),0), max(min(100, dutyCycle - delta),0), 0])
            if(move == "left"):
                motors.move([0, 60, 0, 60])
            if(move == "backward"):
                motors.move([max(min(100, dutyCycle + delta),0), 0, 0, max(min(100, dutyCycle - delta),0)])
            if(move == "right"):
                motors.move([60, 0, 60, 0])
            # motors.move([0,60,60,0])
            sampleTicks += 5
            delta_prev = delta
            errorSum += delta

    motors.move([0, 0, 0, 0])

fig, axs = plt.subplots(2, sharex=True, sharey=True)
axs[0].plot(encoders.left_encoder_count, 'tab:orange', linewidth=1.0)
axs[0].set_title('Left Encoder')
axs[1].plot(encoders.right_encoder_count, 'tab:green', linewidth=1.0)
axs[1].set_title('Right Encoder')
plt.subplots_adjust(top=0.85)
plt.plot()
plt.savefig("p_kp_1")
# while True:
#     print(encoders.get_encoder_ticks())
#     time.sleep(1)

# while(time.time() - start < 2):
#     motors.move([0, 60, 60, 0])
# motors.finish()
# gpio.cleanup()



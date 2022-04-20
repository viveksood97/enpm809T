from gpiozero import Robot, DigitalInputDevice
from time import sleep
import numpy as np

SAMPLETIME = 1
TARGET = 27
KP = 0.1
KD = 0.01

wheel_perimeter = 65*np.pi
ticks_to_travel = (1000/(wheel_perimeter))*20


class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        self.encoder = DigitalInputDevice(pin)
        self.encoder.when_activated = self._increment
        self.encoder.when_deactivated = self._increment
    def reset(self):
        self._value = 0
    def _increment(self):
        self._value += 1
    @property
    def value(self):
        return self._value

r = Robot((19,26), (13,6))

e1 = Encoder(22)
e2 = Encoder(18)

m1_speed = 1.0
m2_speed = 1.0
r.value = (m1_speed, m2_speed)

e1_prev_error = 0
e2_prev_error = 0

total_ticks_e1 = 0
total_ticks_e2 = 0

while(ticks_to_travel > total_ticks_e1 and ticks_to_travel > total_ticks_e1):
    e1_error = TARGET - e1.value
    e2_error = TARGET - e2.value
    
    m1_speed += (e1_error * KP) + (e1_prev_error * KD)
    m2_speed += (e2_error * KP) + (e1_prev_error * KD)

    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)

    r.value = (m1_speed, m2_speed)

    print("e1 {} e2 {}".format(e1.value, e2.value))
    print("m1 {} m2 {}".format(m1_speed, m2_speed))

    e1_prev_error = e1_error
    e2_prev_error = e2_error
    total_ticks_e1 += e1.value
    total_ticks_e2 += e2.value
    e1.reset()
    e2.reset()
    sleep(SAMPLETIME)
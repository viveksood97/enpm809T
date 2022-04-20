from gpiozero import Robot, DigitalInputDevice
from time import sleep
import numpy as np

SAMPLETIME = 1

class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        self.encoder = DigitalInputDevice(pin, pull_up=True)
        self.encoder.when_activated = self._increment
        self.encoder.when_deactivated = self._increment
    def reset(self):
        self._value = 0
    def _increment(self):
        self._value += 1
    @property
    def value(self):
        return self._value

r = Robot((13,6), (19,26))

e1 = Encoder(18)
e2 = Encoder(22)

m1_speed = 0.6
m2_speed = 0.6
r.value = (m1_speed, m2_speed)

TARGET = 24
KP = 0.07
count = 0
ticks = 0

wheel_perimeter = 65*np.pi
ticks_to_travel = (2000/(wheel_perimeter))*20

while(ticks_to_travel > ticks):
    e1_error = TARGET - e1.value
    e2_error = TARGET - e2.value

    m1_speed += e1_error * KP
    m2_speed += e2_error * KP

    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)

    r.value = (m1_speed, m2_speed)

    print("e1 {} e2 {}".format(e1.value, e2.value))
    print("m1 {} m2 {}".format(m1_speed, m2_speed))

    ticks += (e1.value + e2.value)/2

    e1.reset()
    e2.reset()
    sleep(SAMPLETIME)
    


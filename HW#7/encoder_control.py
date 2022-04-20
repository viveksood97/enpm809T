import RPi.GPIO as gpio
import numpy as np
from motorcontrol01 import Motor


counter1 = 0
counter2 = 0
button1 = 0
button2 = 0 
gpio.setmode(gpio.BOARD)
gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
gpio.setup(15, gpio.IN, pull_up_down = gpio.PUD_UP)


motor = Motor()



while(counter1 + counter2 < 40):
    if(int(gpio.input(12)) != button1):
        button1 = int(gpio.input(12))
        counter1 += 1
        print(counter1)

    if(int(gpio.input(15)) != button2):
        button2 = int(gpio.input(15))
        counter2 += 1
        print(counter2)


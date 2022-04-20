import RPi.GPIO as gpio
import time

class Servos:
    def __init__(self, pins, pwm_objects):
        self.pins = pins
        self.pwm_objects = pwm_objects
    

    def finish(self):
        for pin in self.pins:
            gpio.output(pin, False)
        
    def open_gripper(self):
        self.pwm_objects[0].ChangeDutyCycle(9)
       
        

    def close_gripper(self):
        self.pwm_objects[0].ChangeDutyCycle(5)
        
       
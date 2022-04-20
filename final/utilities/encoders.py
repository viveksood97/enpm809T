from random import sample
import RPi.GPIO as gpio
import time


class Encoders:
    def __init__(self):
        self.counter_left = 0
        self.counter_right = 0
        self.previous_left = 0
        self.previous_right = 0
        self.left_encoder_count = []
        self.right_encoder_count = []
        

    def get_encoder_ticks(self):
        self.left_encoder_count.append(int(gpio.input(15)))
        self.right_encoder_count.append(int(gpio.input(12)))
        if(int(gpio.input(15)) != self.previous_left):
            self.previous_left = int(gpio.input(15))
            
            self.counter_left += 1

        if(int(gpio.input(12)) != self.previous_right):
            self.previous_right = int(gpio.input(12))
            
            self.counter_right += 1
        return self.counter_left, self.counter_right
    
    def encoders_reset(self):
        self.counter_left = 0 
        self.counter_right = 0
        

        
    
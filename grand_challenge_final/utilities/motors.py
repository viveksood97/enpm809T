import RPi.GPIO as gpio


class Motors:
    def __init__(self, pins, pwm_objects):
        self.pins = pins
        self.pwm_objects = pwm_objects
    

    def finish(self):
        for pin in self.pins:
            gpio.output(pin, False)
        
    def move(self, outputs):
        for index, output in enumerate(outputs):
                self.pwm_objects[index].ChangeDutyCycle(output)
    




import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(8, GPIO.OUT)

pwm = GPIO.PWM(8, 50)
pwm.start(0)

for i in range(280,450):
    print(i)
    pwm.ChangeDutyCycle(i/100)
    time.sleep(0.1)

pwm.stop()
GPIO.cleanup()
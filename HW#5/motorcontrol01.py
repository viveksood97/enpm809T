import RPi.GPIO as gpio
import time

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)

def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)

def backward(tf):
    init()
    # Left Wheels
    gpio.output(31, True)
    gpio.output(33, False)
    # Right Wheels
    gpio.output(35, False)
    gpio.output(37, True)

    time.sleep(tf)

    gameover()
    gpio.cleanup()

def forward(tf):
    init()
    # Left Wheels
    gpio.output(31, False)
    gpio.output(33, True)
    # Right Wheels
    gpio.output(35, True)
    gpio.output(37, False)

    time.sleep(tf)

    gameover()
    gpio.cleanup()

def pivotLeft(tf):
    init()
    # Left Wheels
    gpio.output(31, True)
    gpio.output(33, False)
    # Right Wheels
    gpio.output(35, True)
    gpio.output(37, False)

    time.sleep(tf)

    gameover()
    gpio.cleanup()

def pivotRight(tf):
    init()
    # Left Wheels
    gpio.output(31, False)
    gpio.output(33, True)
    # Right Wheels
    gpio.output(35, False)
    gpio.output(37, True)

    time.sleep(tf)

    gameover()
    gpio.cleanup()

# backward(2)
# forward(2)
#pivotLeft(2)

def key_input(event):
    init()
    print("Key: ", event)
    key_press = event
    tf = 1

    if key_press.lower() == 'w':
        forward(tf)
    elif key_press.lower() == 's':
        backward(tf)
    elif key_press.lower() == 'a':
        pivotLeft(tf)
    elif key_press.lower() == 'd':
        pivotRight(tf)
    else:
        print("Invalid Key pressed")

while True:
    key_press = input("Direction please: ")
    if key_press == 'p':
        break
    key_input(key_press)
    

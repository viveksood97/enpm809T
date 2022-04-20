import RPi.GPIO as gpio

class Initialize:
    def __init__(self):
        gpio.setmode(gpio.BOARD)
        self.motor_pins = [31, 33, 35, 37]
        self.motor_pwm_objects = []

        self.servo_pins = [8]
        self.motor_pwm_objects = []

    def init_board_pins(self):
        #motor
        gpio.setup(31, gpio.OUT)
        gpio.setup(33, gpio.OUT)
        gpio.setup(35, gpio.OUT)
        gpio.setup(37, gpio.OUT)

        #encoder
        gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
        gpio.setup(15, gpio.IN, pull_up_down = gpio.PUD_UP)

        #gripper
        gpio.setup(8, gpio.OUT)

        #ultrasonic_sensor
        gpio.setup(16, gpio.OUT)
        gpio.setup(18, gpio.IN)

    
    def init_bcm_pins(self):
        pass

    def set_motor_pins(self):
        pwm_right_one = gpio.PWM(31, 100)
        pwm_right_two = gpio.PWM(33, 100)
        pwm_left_one = gpio.PWM(35, 100)
        pwm_left_two = gpio.PWM(37, 100)

        pwm_right_one.start(0)
        pwm_right_two.start(0)
        pwm_left_one.start(0)
        pwm_left_two.start(0)

        self.motor_pwm_objects = [pwm_right_one, pwm_right_two, pwm_left_one, pwm_left_two]
    
    def set_servo_pins(self):
        pwm_gripper = gpio.PWM(8, 50)
        pwm_gripper.start(0)

        self.servo_pwm_objects = [pwm_gripper]

    def startup(self):
        self.init_board_pins()
        self.init_bcm_pins()
        self.set_motor_pins()
        self.set_servo_pins()
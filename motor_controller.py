import time
import RPi.GPIO as GPIO


class MotorController:
    '''Motor controller'''
    # ENA, ENB
    EN_A, EN_B = 18, 12

    #        Derecha                    Izquierda
    #   IN1           IN2           IN3           IN4
    CONTROLLER_1, CONTROLLER_2, CONTROLLER_3, CONTROLLER_4 = 4, 17, 27, 22

    # Frequency
    FREQUENCY = 50
    
    # Duty Cycle
    FORWARD_DUTY_CYCLE = 40

    FAST_TURN_DUTY_CYCLE = 80
    SLOW_TURN_DUTY_CYCLE = 10

    ONLY_TURN_DUTY_CYCLE = 90

    def __init__(self, gpio_mode):
        '''Setup motor controller'''
        GPIO.setmode(gpio_mode)
        GPIO.setup(MotorController.EN_A, GPIO.OUT)
        GPIO.setup(self.EN_B, GPIO.OUT)
        GPIO.setup(self.CONTROLLER_1, GPIO.OUT)
        GPIO.setup(self.CONTROLLER_2, GPIO.OUT)
        GPIO.setup(self.CONTROLLER_3, GPIO.OUT)
        GPIO.setup(self.CONTROLLER_4, GPIO.OUT)
        self.en_a = GPIO.PWM(self.EN_A, MotorController.FREQUENCY)
        self.en_b = GPIO.PWM(self.EN_B, MotorController.FREQUENCY)
        self.en_a.start(MotorController.FORWARD_DUTY_CYCLE)
        self.en_b.start(MotorController.FORWARD_DUTY_CYCLE)

    def forward(self, sleep_time):
        '''Motors forward'''
        self.en_a.ChangeDutyCycle(MotorController.FORWARD_DUTY_CYCLE)
        self.en_b.ChangeDutyCycle(MotorController.FORWARD_DUTY_CYCLE)
        GPIO.output(MotorController.CONTROLLER_1, False)
        GPIO.output(MotorController.CONTROLLER_2, True)
        GPIO.output(MotorController.CONTROLLER_3, False)
        GPIO.output(MotorController.CONTROLLER_4, True)
        time.sleep(sleep_time)

    def forward_left(self, sleep_time):
        '''Motors forward left'''
        self.en_a.ChangeDutyCycle(MotorController.FAST_TURN_DUTY_CYCLE)
        self.en_b.ChangeDutyCycle(MotorController.SLOW_TURN_DUTY_CYCLE)
        GPIO.output(MotorController.CONTROLLER_1, False)
        GPIO.output(MotorController.CONTROLLER_2, True)
        GPIO.output(MotorController.CONTROLLER_3, False)
        GPIO.output(MotorController.CONTROLLER_4, True)
        time.sleep(sleep_time)

    def forward_right(self, sleep_time):
        '''Motors forward right'''
        self.en_a.ChangeDutyCycle(MotorController.SLOW_TURN_DUTY_CYCLE)
        self.en_b.ChangeDutyCycle(MotorController.FAST_TURN_DUTY_CYCLE)
        GPIO.output(MotorController.CONTROLLER_1, False)
        GPIO.output(MotorController.CONTROLLER_2, True)
        GPIO.output(MotorController.CONTROLLER_3, False)
        GPIO.output(MotorController.CONTROLLER_4, True)
        time.sleep(sleep_time)

    def reverse(self, sleep_time):
        '''Motors in reverse'''
        self.en_a.ChangeDutyCycle(MotorController.FORWARD_DUTY_CYCLE)
        self.en_b.ChangeDutyCycle(MotorController.FORWARD_DUTY_CYCLE)
        GPIO.output(MotorController.CONTROLLER_1, True)
        GPIO.output(MotorController.CONTROLLER_2, False)
        GPIO.output(MotorController.CONTROLLER_3, True)
        GPIO.output(MotorController.CONTROLLER_4, False)
        time.sleep(sleep_time)

    def turn_right(self, sleep_time):
        '''Motors turn right'''
        self.en_a.ChangeDutyCycle(MotorController.ONLY_TURN_DUTY_CYCLE)
        self.en_b.ChangeDutyCycle(MotorController.ONLY_TURN_DUTY_CYCLE)
        GPIO.output(MotorController.CONTROLLER_1, True)
        GPIO.output(MotorController.CONTROLLER_2, False)
        GPIO.output(MotorController.CONTROLLER_3, False)
        GPIO.output(MotorController.CONTROLLER_4, True)
        time.sleep(sleep_time)

    def turn_left(self, sleep_time):
        '''Motors turn left'''
        self.en_a.ChangeDutyCycle(MotorController.ONLY_TURN_DUTY_CYCLE)
        self.en_b.ChangeDutyCycle(MotorController.ONLY_TURN_DUTY_CYCLE)
        GPIO.output(MotorController.CONTROLLER_1, False)
        GPIO.output(MotorController.CONTROLLER_2, True)
        GPIO.output(MotorController.CONTROLLER_3, True)
        GPIO.output(MotorController.CONTROLLER_4, False)
        time.sleep(sleep_time)

    def stop(self, sleep_time):
        '''Motors stop'''
        GPIO.output(MotorController.CONTROLLER_1, False)
        GPIO.output(MotorController.CONTROLLER_2, False)
        GPIO.output(MotorController.CONTROLLER_3, False)
        GPIO.output(MotorController.CONTROLLER_4, False)
        time.sleep(sleep_time)

    def exit(self):
        '''Exit motor'''
        self.en_a.ChangeDutyCycle(0)
        self.en_b.ChangeDutyCycle(0)
        self.en_a.stop()
        self.en_b.stop()

    def calculate_speed(self, distance):
        '''Calculate the speed based on the distance to an object'''
        speed = distance * 1.2
        print("Speed: {}".format(speed))
        if speed > 100:
            self.en_a.ChangeDutyCycle(100)
            self.en_b.ChangeDutyCycle(100)
        else:
            self.en_a.ChangeDutyCycle(speed)
            self.en_b.ChangeDutyCycle(speed)

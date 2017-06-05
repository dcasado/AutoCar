import time
import RPi.GPIO as GPIO


class MotorController:
    '''Motor controller'''
    # ENA, ENB
    EN_A, EN_B = 18, 12

    # Derecha
    # IN1
    CONTROLLER_1 = 4
    # IN2
    CONTROLLER_2 = 17
    # Izquierda
    # IN3
    CONTROLLER_3 = 27
    # IN4
    CONTROLLER_4 = 22

    # Frequency
    FREQUENCY = 50
    # Duty Cycle
    DUTY_CYCLE = 75

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
        self.en_a.start(MotorController.DUTY_CYCLE)
        self.en_b.start(MotorController.DUTY_CYCLE)

    def forward(self, sleep_time):
        '''Motors forward'''
        GPIO.output(MotorController.CONTROLLER_1, False)
        GPIO.output(MotorController.CONTROLLER_2, True)
        GPIO.output(MotorController.CONTROLLER_3, False)
        GPIO.output(MotorController.CONTROLLER_4, True)
        time.sleep(sleep_time)

    def reverse(self, sleep_time):
        '''Motors in reverse'''
        GPIO.output(MotorController.CONTROLLER_1, True)
        GPIO.output(MotorController.CONTROLLER_2, False)
        GPIO.output(MotorController.CONTROLLER_3, True)
        GPIO.output(MotorController.CONTROLLER_4, False)
        time.sleep(sleep_time)

    def turn_right(self, sleep_time):
        '''Motors turn right'''
        GPIO.output(MotorController.CONTROLLER_1, True)
        GPIO.output(MotorController.CONTROLLER_2, False)
        GPIO.output(MotorController.CONTROLLER_3, False)
        GPIO.output(MotorController.CONTROLLER_4, True)
        time.sleep(sleep_time)

    def turn_left(self, sleep_time):
        '''Motors turn left'''
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

    def finish(self):
        '''Finish motor'''
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

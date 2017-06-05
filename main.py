import time
from enum import Enum
from ultrasonic_sensor import UltrasonicSensor
from motor_controller import MotorController as Motor
import RPi.GPIO as GPIO


class States(Enum):
    '''States enumerable'''
    FORWARD = 0
    STOP = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    REVERSE = 4


class Main:
    '''Main class'''
    CONTROLLER_SLEEP_TIME = 0.060
    STOP_DISTANCE = 20  # Distance from object to stop

    GPIO_MODE = GPIO.BCM

    def __init__(self):
        # Setup the needed components
        self.motor = Motor(Main.GPIO_MODE)
        self.sensor = UltrasonicSensor(Main.GPIO_MODE)
        self.state = None
        print("Setup finished")

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.motor.finish()
        # Clean GPIO
        GPIO.cleanup()

    def motor_forward(self):
        '''Makes the car go forward'''
        print("Motor forward")
        self.motor.forward(Main.CONTROLLER_SLEEP_TIME)
        self.state = States.FORWARD

    def motor_stop(self):
        '''Makes the car stop'''
        print("Motor stop")
        self.motor.stop(Main.CONTROLLER_SLEEP_TIME)
        self.state = States.STOP

    def motor_reverse(self):
        '''Makes the car go reverse'''
        print("Motor reverse")
        self.motor.reverse(Main.CONTROLLER_SLEEP_TIME)
        self.state = States.REVERSE

    def motor_turn_left(self):
        '''Makes the car turn left'''
        print("Motor turn left")
        self.motor.turn_left(Main.CONTROLLER_SLEEP_TIME)
        self.state = States.TURN_LEFT

    def motor_turn_right(self):
        '''Makes the car turn right'''
        print("Motor turn right")
        self.motor.turn_right(Main.CONTROLLER_SLEEP_TIME)
        self.state = States.TURN_RIGHT

    def choose_mode(self):
        '''Choose drive mode'''
        print("Choose operation mode: manual(m)/auto(a): ")
        if input("> ") == "m":
            self.manual_mode()
        else:
            self.auto_mode()

    def manual_mode(self):
        '''Manual mode'''
        while True:
            try:
                print("Choose a command: w/a/s/d/p")
                command = input("> ")
                if command == "w":
                    self.motor_forward()
                elif command == "s":
                    self.motor_reverse()
                elif command == "a":
                    self.motor_turn_left()
                elif command == "d":
                    self.motor_turn_right()
                elif command == "p":
                    self.motor_stop()
                else:
                    pass
            except KeyboardInterrupt:
                break

    def auto_mode(self):
        '''Auto mode'''
        # Go forward until find an obstacle
        self.motor_forward()
        while True:
            try:
                distance = self.sensor.get_distance3()
                print("Distance: {} cm, State: {}".format(distance, self.state))
                self.motor.calculate_speed(distance)
                if distance <= Main.STOP_DISTANCE:
                    if self.state is States.FORWARD:
                        self.motor_reverse()
                        time.sleep(0.1)
                    elif self.state is States.STOP:
                        #print("Motor already stopped")
                        pass
                    elif self.state is States.TURN_LEFT:
                        self.motor_stop()
                    elif self.state is States.TURN_RIGHT:
                        self.motor_stop()
                    else:
                        self.motor_stop()
                else:
                    if self.state is States.STOP:
                        self.motor_forward()
                    elif self.state is States.REVERSE:
                        self.motor_stop()
                #print("New state: ", state)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    with Main() as main:
        main.motor_stop()
        main.choose_mode()

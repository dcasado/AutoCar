import time
from enum import Enum
from ultrasonic_sensor import UltrasonicSensor
from motor_controller import MotorController as Motor
from camera import Camera
import numpy as np
import cv2
import RPi.GPIO as GPIO


class States(Enum):
    '''States enumerable'''
    FORWARD = 0
    STOP = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    REVERSE = 4


class MainController:
    '''MainController class'''
    # Anterior sleep time 0.06
    CONTROLLER_SLEEP_TIME = 0.06
    STOP_DISTANCE = 25  # Distance from object to stop

    GPIO_MODE = GPIO.BCM

    VERTICES = np.array([[0, 240], [0, 140], [100, 100],
                             [220, 100], [320, 140], [320, 240]])

    def __init__(self):
        # Setup the needed components
        self.motor = Motor(MainController.GPIO_MODE)
        self.sensor = UltrasonicSensor(MainController.GPIO_MODE)
        self.camera = Camera()
        self.state = None
        print("Setup finished")

    def exit(self):
        '''Exit main closing all'''
        self.motor.exit()
        self.camera.close()
        # Clean GPIO
        GPIO.cleanup()

    def motor_forward(self, sleep_time):
        '''Makes the car go forward'''
        print("Motor forward")
        self.motor.forward(sleep_time)
        self.state = States.FORWARD

    def motor_forward_left(self, sleep_time):
        '''Makes the car go forward and left'''
        print("Motor forward left")
        self.motor.forward_left(sleep_time)
        self.state = States.FORWARD

    def motor_forward_right(self, sleep_time):
        '''Makes the car go forward and right'''
        print("Motor forward left")
        self.motor.forward_right(sleep_time)
        self.state = States.FORWARD

    def motor_stop(self, sleep_time):
        '''Makes the car stop'''
        print("Motor stop")
        self.motor.stop(sleep_time)
        self.state = States.STOP

    def motor_reverse(self, sleep_time):
        '''Makes the car go reverse'''
        print("Motor reverse")
        self.motor.reverse(sleep_time)
        self.state = States.REVERSE

    def motor_turn_left(self, sleep_time):
        '''Makes the car turn left'''
        print("Motor turn left")
        self.motor.turn_left(sleep_time)
        self.state = States.TURN_LEFT

    def motor_turn_right(self, sleep_time):
        '''Makes the car turn right'''
        print("Motor turn right")
        self.motor.turn_right(sleep_time)
        self.state = States.TURN_RIGHT

    def choose_mode(self):
        '''Choose drive mode'''
        self.motor_stop(0)
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
                # self.motor.calculate_speed(100)
                if command == "w":
                    self.motor_forward(0)
                elif command == "s":
                    self.motor_reverse(0)
                elif command == "a":
                    self.motor_turn_left(0)
                elif command == "d":
                    self.motor_turn_right(0)
                elif command == "p":
                    self.motor_stop(0)
                else:
                    pass
            except KeyboardInterrupt:
                break

    def auto_mode(self):
        '''Auto mode'''
        # Go forward until find an obstacle
        # self.motor_forward()
        start_time = time.time()
        while True:
            try:
                distance = self.sensor.get_distance3()
                print("Distance:", distance, "cm, State:", self.state)
                # self.motor.calculate_speed(distance)
                if distance <= MainController.STOP_DISTANCE:
                    if self.state is States.FORWARD:
                        self.motor_reverse(MainController.CONTROLLER_SLEEP_TIME * 4)
                    self.motor_stop(0)
                else:
                    positive_line, negative_line = self.camera.get_processed_frame(MainController.VERTICES, True)
                    if positive_line["slope"] != -1 and negative_line["slope"] != -1:
                        print("Go straight")
                        self.motor_forward(0)
                    elif positive_line["slope"] == -1 and negative_line["slope"] != -1:
                        print("Turn right")
                        self.motor_forward_left(MainController.CONTROLLER_SLEEP_TIME)
                        # self.motor_turn_right()
                    elif positive_line["slope"] != -1 and negative_line["slope"] == -1:
                        print("Turn left")
                        self.motor_forward_right(MainController.CONTROLLER_SLEEP_TIME)
                        # self.motor_turn_left()
                    else:
                        print("Go anywhere")
                        self.motor_forward(0)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                print("Frame took ", time.time() - start_time)
                start_time = time.time()

            except KeyboardInterrupt:
                break


def main():
    '''Main function'''
    try:
        main_controller = MainController()
        main_controller.choose_mode()
    finally:
        main_controller.exit()

if __name__ == "__main__":
    main()

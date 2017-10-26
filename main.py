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
    FORWARD_LEFT = 5
    FORWARD_RIGHT = 6


class MainController:
    '''MainController class'''
    # Anterior sleep time 0.06
    CONTROLLER_SLEEP_TIME = 0.0
    STOP_DISTANCE = 25  # Distance from object to stop

    GPIO_MODE = GPIO.BCM

    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240

    DISPLAY_IMAGE = True

    def __init__(self):
        # Setup the needed components
        self.state = None
        self.motor = Motor(MainController.GPIO_MODE)
        self.motor_stop(0)
        self.sensor = UltrasonicSensor(MainController.GPIO_MODE).start()
        self.camera = Camera(width=MainController.CAMERA_WIDTH, height=MainController.CAMERA_HEIGHT,
            display_images=MainController.DISPLAY_IMAGE)
        print("Setup finished")

    def exit(self):
        '''Exit main closing all'''
        self.motor.exit()
        self.camera.close()
        # Clean GPIO
        GPIO.cleanup()

    def motor_forward(self, sleep_time):
        '''Makes the car go forward'''
        if self.state is not States.FORWARD:
            print("Motor forward")
            self.motor.forward(sleep_time)
            self.state = States.FORWARD

    def motor_forward_left(self, sleep_time):
        '''Makes the car go forward and left'''
        if self.state is not States.FORWARD_LEFT:
            print("Motor forward left")
            self.motor.forward_left(sleep_time)
            self.state = States.FORWARD_LEFT

    def motor_forward_right(self, sleep_time):
        '''Makes the car go forward and right'''
        if self.state is not States.FORWARD_RIGHT:
            print("Motor forward left")
            self.motor.forward_right(sleep_time)
            self.state = States.FORWARD_RIGHT

    def motor_stop(self, sleep_time):
        '''Makes the car stop'''
        if self.state is not States.STOP:
            print("Motor stop")
            self.motor.stop(sleep_time)
            self.state = States.STOP

    def motor_reverse(self, sleep_time):
        '''Makes the car go reverse'''
        if self.state is not States.REVERSE:
            print("Motor reverse")
            self.motor.reverse(sleep_time)
            self.state = States.REVERSE

    def motor_turn_left(self, sleep_time):
        '''Makes the car turn left'''
        if self.state is not States.TURN_LEFT:
            print("Motor turn left")
            self.motor.turn_left(sleep_time)
            self.state = States.TURN_LEFT

    def motor_turn_right(self, sleep_time):
        '''Makes the car turn right'''
        if self.state is not States.TURN_RIGHT:
            print("Motor turn right")
            self.motor.turn_right(sleep_time)
            self.state = States.TURN_RIGHT

    def choose_mode(self):
        '''Choose drive mode'''
        print("Choose operation mode: manual(m)/auto(a) or exit: ")
        good_option = False
        while not good_option:
            command = input("> ")
            if command == "m":
                good_option = True
                self.manual_mode()
            elif command == "a":
                good_option = True
                self.auto_mode()
            elif command == "exit":
                break
            else:
                pass


    def manual_mode(self):
        '''Manual mode'''
        try:
            while True:
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
                elif command == "exit":
                    self.motor_stop(0)
                    self.choose_mode()
                    break
                else:
                    pass
        except KeyboardInterrupt:
            pass

    def auto_mode(self):
        '''Auto mode'''
        # Go forward until find an obstacle
        # self.motor_forward()
        start_time = time.time()
        try:
            while True:
                distance = self.sensor.read_distance()
                # self.motor.calculate_speed(distance)
                # print("Distance:", distance, "cm, State:", self.state)
                if distance <= MainController.STOP_DISTANCE:
                    if self.state is States.FORWARD:
                        self.motor_reverse(
                            MainController.CONTROLLER_SLEEP_TIME * 2)
                    self.motor_stop(0)
                else:
                    positive_line, negative_line = self.camera.get_processed_frame()
                    positive_slope = positive_line["slope"]
                    negative_slope = negative_line["slope"]
                    # Two lines found
                    if positive_slope is not None and negative_slope is not None:
                        #self.motor_forward(0)
                        self.maintain_middle(positive_line, negative_line)
                    # Right line found
                    elif positive_slope is None and negative_slope is not None:
                        self.motor_forward_left(
                            MainController.CONTROLLER_SLEEP_TIME)
                    # Left line found
                    elif positive_slope is not None and negative_slope is None:
                        self.motor_forward_right(
                            MainController.CONTROLLER_SLEEP_TIME)
                    # No lines found
                    else:
                        self.motor_forward(0)
                    if self.DISPLAY_IMAGE:
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                print("Frame took ", time.time() - start_time)
                start_time = time.time()
        except KeyboardInterrupt:
            self.motor_stop(0)
            self.choose_mode()


    def maintain_middle(self, positive_line, negative_line):
        '''Maintain the car in the middle of the road'''
        # print(("Middle screen ", self.camera.camera_width/2), " middle point between lines ", ((positive_line["points"][0] + negative_line["points"][2])/2))
        if (self.camera.camera_width/2) < ((positive_line["points"][0] + negative_line["points"][2])/2):
            self.motor_forward_left(MainController.CONTROLLER_SLEEP_TIME)
        else:
            self.motor_forward_right(MainController.CONTROLLER_SLEEP_TIME)

def main():
    '''Main function'''
    try:
        main_controller = MainController()
        main_controller.choose_mode()
    finally:
        main_controller.exit()


if __name__ == "__main__":
    main()

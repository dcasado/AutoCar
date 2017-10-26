import time
import functools
from threading import Thread
import RPi.GPIO as GPIO


class UltrasonicSensor:
    '''Ultrasonic sensor'''
    TRIG = 23
    ECHO = 24

    TIME_CONSTANT = 58

    NUM_READINGS = 3

    def __init__(self, gpio_mode):
        '''Setup ultrasonic sensor'''
        GPIO.setmode(gpio_mode)
        GPIO.setup(UltrasonicSensor.TRIG, GPIO.OUT)
        GPIO.setup(UltrasonicSensor.ECHO, GPIO.IN)
        self.readings = []
        self.read_index = 0
        self.total = 0
        self.readings = [0] * UltrasonicSensor.NUM_READINGS
        self.distance = 0
        self.stopped = False

    def start(self):
        '''Start calculating distances'''
        thread = Thread(target=self.calculate_distance, args=())
        thread.daemon = True
        thread.start()
        return self

    def calculate_distance(self):
        '''Get distance from sensor'''
        try:
            while not self.stopped:
                reading_error = False
                reading = 0
                last_reading = self.readings[self.read_index]

                # subtract the last reading:
                self.total -= self.readings[self.read_index]

                # If the 'Echo' pin is already high
                if GPIO.input(UltrasonicSensor.ECHO):
                    # then set reading_error to True
                    reading_error = True

                # Ensure the 'Trig' pin is low for at least 50mS (recommended re-sample
                # time)
                GPIO.output(UltrasonicSensor.TRIG, False)
                # Default value 0.05
                time.sleep(0.04)
                # Turn on the 'Trig' pin for 10uS (ish!)
                GPIO.output(UltrasonicSensor.TRIG, True)
                time.sleep(10 * 10**-6)
                # Turn off the 'Trig' pin
                GPIO.output(UltrasonicSensor.TRIG, False)
                # Set inital time values to current time
                time1, time2 = time.time(), time.time()

                # Wait for the start of the 'Echo' pulse
                while not GPIO.input(UltrasonicSensor.ECHO) and not reading_error:
                    # Get the time the 'Echo' pin goes high
                    time1 = time.time()
                    # If the 'Echo' pin doesn't go high after 20mS
                    if time1 - time2 > 0.02:
                        # then set reading_error to True and break out of the loop
                        # print("Reading error")
                        reading_error = True
                        break

                # Otherwise, wait for the 'Echo' pin to go low
                while GPIO.input(UltrasonicSensor.ECHO) and not reading_error:
                    # Get the time the 'Echo' pin goes low
                    time2 = time.time()
                    # If the 'Echo' pin doesn't go low after 20mS
                    if time2 - time1 > 0.02:
                        # then set reading_error to True and break out of the loop
                        # print("Reading error")
                        reading_error = True
                        break

                # print "Last reading: ", last_reading

                if not reading_error:
                    reading = time2 - time1
                    # print "Reading: ", reading
                else:
                    reading = last_reading
                    # print "Error reading: ", reading

                self.readings[self.read_index] = reading
                self.total += self.readings[self.read_index]

                # print "Total post: ", TOTAL

                self.read_index += 1
                if self.read_index >= UltrasonicSensor.NUM_READINGS:
                    # ...wrap around to the beginning:
                    self.read_index = 0

                average = self.total / UltrasonicSensor.NUM_READINGS
                # Sound travels at approximately 2.95uS per mm
                # and the reflected sound has travelled twice
                # the distance we need to measure (sound out, bounced off object, sound returned)
                # Convert the timer values into centimetres
                self.distance = average / 0.00000295 / 2 / 10
        except RuntimeError:
            pass

    def read_distance(self):
        '''Read the last distance'''
        return self.distance

    def stop(self):
        '''Stop reading values'''
        self.stopped = True

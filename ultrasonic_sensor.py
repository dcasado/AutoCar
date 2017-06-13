import time
import functools
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

    def get_distance3(self):
        '''Get distance from sensor'''
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
        distance = average / 0.00000295 / 2 / 10
        return distance

    def get_distance2(self):
        '''Get distance of sensor'''
        # If the 'Echo' pin is already high
        if GPIO.input(UltrasonicSensor.ECHO):
            # then exit with 100 (sensor fault)
            return 100

        # Set initial distance to zero
        distance = 0

        # Ensure the 'Trig' pin is low for at least 50mS (recommended re-sample
        # time)
        GPIO.output(UltrasonicSensor.TRIG, False)
        time.sleep(0.05)
        # Turn on the 'Trig' pin for 10uS (ish!)
        GPIO.output(UltrasonicSensor.TRIG, True)
        time.sleep(10 * 10**-6)
        # Turn off the 'Trig' pin
        GPIO.output(UltrasonicSensor.TRIG, False)
        # Set inital time values to current time
        time1, time2 = time.time(), time.time()

        # Wait for the start of the 'Echo' pulse
        while not GPIO.input(UltrasonicSensor.ECHO):
            # Get the time the 'Echo' pin goes high
            time1 = time.time()
            # If the 'Echo' pin doesn't go high after 20mS
            if time1 - time2 > 0.02:
                # then set 'distance' to 100
                distance = 100
                # and break out of the loop
                break
        # If a sensor error has occurred
        if distance == 100:
            # then exit with 100 (sensor fault)
            return distance

        # Otherwise, wait for the 'Echo' pin to go low
        while GPIO.input(UltrasonicSensor.ECHO):
            # Get the time the 'Echo' pin goes low
            time2 = time.time()
            # If the 'Echo' pin doesn't go low after 20mS
            if time2 - time1 > 0.02:
                # then ignore it and set 'distance' to 100
                distance = 100
                # and break out of the loop
                break
        # If a sensor error has occurred
        if distance == 100:
            # then exit with 100 (sensor fault)
            return distance

        # Sound travels at approximately 2.95uS per mm
        # and the reflected sound has travelled twice
        # the distance we need to measure (sound out, bounced off object, sound returned)
        # Convert the timer values into centimetres
        distance = (time2 - time1) / 0.00000295 / 2 / 10
        return distance

    def get_distance(self):
        '''Returns distance to object'''
        count = 0
        array_distance = []
        while count < UltrasonicSensor.NUM_READINGS:
            start = 0
            end = 0

            GPIO.output(UltrasonicSensor.TRIG, False)  # Turn off trig
            time.sleep(2 * 10**-6)  # Wait 2 microseconds
            GPIO.output(UltrasonicSensor.TRIG, True)  # Turn on trig
            time.sleep(10 * 10**-6)  # Wait 10 microseconds
            GPIO.output(UltrasonicSensor.TRIG, False)  # Turn off trig

            # Start time when echo turns on
            while GPIO.input(UltrasonicSensor.ECHO) == 0:
                # print "Waiting ECHO 0"
                start = time.time()
            while GPIO.input(UltrasonicSensor.ECHO) == 1:
                # print "Waiting ECHO 1"
                end = time.time()

            # Compute the duration
            duration = end - start

            # Compute de distance in cm
            array_distance.append(
                (duration * 10**6 / UltrasonicSensor.TIME_CONSTANT) * 0.95)
            # print arrayDistance
            count += 1

        # Remove max and min to better compute the mean
        array_distance.remove(max(array_distance))
        array_distance.remove(min(array_distance))

        # Compute mean
        mean_distance = functools.reduce(
            lambda x, y: x + y, array_distance) / len(array_distance)
        return mean_distance

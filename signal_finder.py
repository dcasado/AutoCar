import random
import cv2
import numpy as np
from utils import display_image

class SignalFinder:
    '''Class to find signal in image'''

    # define the list of boundaries: hue, saturation, value
    RED_LOWER_BOUNDARY = ([0, 180, 25], [5, 255, 125])
    RED_UPPER_BOUNDARY = ([150, 160, 25], [179, 255, 125])
    GREEN_BOUNDARY = ([70, 240, 5], [90, 255, 80])
    BLUE_BOUNDARY = ([111, 240, 20], [118, 255, 135])

    def divide_by_color(self, frame):
        '''Divide frame by color'''

        # frame = cv2.GaussianBlur(frame, (3, 3), 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # create NumPy arrays from the boundaries
        lower = np.array(self.RED_LOWER_BOUNDARY[0], dtype="uint8")
        upper = np.array(self.RED_LOWER_BOUNDARY[1], dtype="uint8")

        # find the colors within the specified boundaries and apply
        # the mask
        red_lower_mask = cv2.inRange(frame, lower, upper)

        lower = np.array(self.RED_UPPER_BOUNDARY[0], dtype="uint8")
        upper = np.array(self.RED_UPPER_BOUNDARY[1], dtype="uint8")
        red_upper_mask = cv2.inRange(frame, lower, upper)

        # cv2.imshow("red lower", red_lower_mask)
        # cv2.imshow("red upper", red_upper_mask)

        red_mask = cv2.add(red_lower_mask, red_upper_mask)
        # red_mask = red_upper_mask

        red_mask = cv2.erode(red_mask, None, iterations=2)
        red_mask = cv2.dilate(red_mask, None, iterations=2)
        red_output = cv2.bitwise_and(frame, frame, mask=red_mask)

        lower = np.array(self.GREEN_BOUNDARY[0], dtype="uint8")
        upper = np.array(self.GREEN_BOUNDARY[1], dtype="uint8")

        green_mask = cv2.inRange(frame, lower, upper)
        green_mask = cv2.erode(green_mask, None, iterations=2)
        green_mask = cv2.dilate(green_mask, None, iterations=2)
        green_output = cv2.bitwise_and(frame, frame, mask=green_mask)

        lower = np.array(self.BLUE_BOUNDARY[0], dtype="uint8")
        upper = np.array(self.BLUE_BOUNDARY[1], dtype="uint8")

        blue_mask = cv2.inRange(frame, lower, upper)
        blue_mask = cv2.erode(blue_mask, None, iterations=2)
        blue_mask = cv2.dilate(blue_mask, None, iterations=2)
        blue_output = cv2.bitwise_and(frame, frame, mask=blue_mask)

        cnts = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL,
	                               cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
			    # draw the circle and centroid on the frame,
			    # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # show the images
        cv2.imshow("images", np.hstack([cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)]))
        display_image(red_mask, "red mask")
        #     cv2.cvtColor(green_output, cv2.COLOR_HSV2BGR),
        #     cv2.cvtColor(blue_output, cv2.COLOR_HSV2BGR)]))

    def get_signals(self, frame):
        '''Get the signals in the image'''
        self.divide_by_color(frame)
        return random.random()

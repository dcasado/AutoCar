import cv2
import numpy as np

class SignalFinder:
    '''Class to find signal in image'''

    # define the list of boundaries: red, green, blue and grey
    # (taking into account that are in code BGR rather than RGB)
    BOUNDARIES = [
        ([17, 15, 100], [50, 56, 200])
        # ([86, 31, 4], [220, 88, 50]),
        # ([25, 146, 190], [62, 174, 250]),
        # ([103, 86, 65], [145, 133, 128])
        ]

    def divide_by_color(self, frame):
        '''Divide frame by color'''
        # loop over the boundaries
        for (lower, upper) in self.BOUNDARIES:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(frame, lower, upper)
            output = cv2.bitwise_and(frame, frame, mask=mask)

            # show the images
            cv2.imshow("images", np.hstack([frame, output]))

    def get_signals(self, frame):
        '''Get the signals in the image'''
        self.divide_by_color(frame)
        return 0

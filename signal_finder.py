import cv2
import numpy as np
from utils import display_image, draw_contour_bounding_box

class SignalFinder:
    '''Class to find signal in image'''

    # define the list of boundaries: hue, saturation, value
    RED_LOWER_BOUNDARY = ([0, 120, 70], [15, 215, 155])
    RED_UPPER_BOUNDARY = ([165, 60, 45], [179, 165, 185])
    GREEN_BOUNDARY = ([70, 240, 5], [90, 255, 255])
    BLUE_BOUNDARY = ([105, 200, 15], [120, 255, 255])

    KERNEL = np.ones((3, 3), np.uint8)

    def __init__(self, cascade_classifier):
        self.cascade_classifier = cv2.CascadeClassifier(cascade_classifier)

    def detect_signals(self, frame):
        '''Detect signals'''
        red_candidates = self.get_signals_by_color(frame)
        cropped_frames = self.crop_candidates(frame, red_candidates)
        self.get_signals_with_cascade(cropped_frames)
        # self.get_signals_by_edge(cropped_frames)

    def crop_candidates(self, frame, candidates):
        '''Crop images from a frame'''
        rectangles = []
        cropped_frames = []
        for candidate in candidates:
            padding = 20
            x_rect, y_rect, w_rect, h_rect = candidate
            if y_rect - padding > 0:
                y_rect_1 = y_rect - padding
            else:
                y_rect_1 = y_rect
            if x_rect - padding > 0:
                x_rect_1 = x_rect - padding
            else:
                x_rect_1 = x_rect
            # Double because if not group rectangles does not work
            rectangles.append((x_rect_1, y_rect_1, w_rect + padding, h_rect + padding))
            rectangles.append((x_rect_1, y_rect_1, w_rect + padding, h_rect + padding))
            # cv2.rectangle(frame, (x_rect_1, y_rect_1), (x_rect + w_rect + padding, y_rect + h_rect + padding), (0, 255, 0), 2)
            # cropped_frames.append(frame[y_rect_1:y_rect + h_rect+padding, x_rect_1:x_rect + w_rect+padding])
        # Group rectangles that are touching
        rectangles, weights = cv2.groupRectangles(rectangles, 1, 1)
        for (x_rect, y_rect, w_rect, h_rect) in rectangles:
            cv2.rectangle(frame, (x_rect, y_rect), (x_rect + w_rect + padding, y_rect + h_rect + padding), (0, 0, 255), 2)
            cropped_frames.append(frame[y_rect:y_rect + h_rect+padding, x_rect:x_rect + w_rect + padding])

        return cropped_frames

    def get_signals_with_cascade(self, frames):
        '''Detect objects with cascade classifier'''
        for frame in frames:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.equalizeHist(frame)
            rects = self.cascade_classifier.detectMultiScale(frame, 1.06, 1)
            # for (x_rect, y_rect, w_rect, h_rect) in rects:
            #     cv2.rectangle(orig_frame, (x_rect, y_rect), (x_rect + w_rect, y_rect + h_rect), (255, 0, 0), 2)

    def get_signals_by_edge(self, frames):
        for index, frame in enumerate(frames):
            # Calculate edges
            m_frame = cv2.GaussianBlur(frame, (5, 5), 0)
            m_frame = cv2.Canny(m_frame, 50, 100, apertureSize=3)
            # display_image(m_frame, str(index))
            contours = self.find_contours(m_frame)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:
                # moments = cv2.moments(contour)
                # center_x = int((moments["m10"] / moments["m00"]))
                # center_y = int((moments["m01"] / moments["m00"]))
                    draw_contour_bounding_box(frame, contour)
                    approx_points = self.get_contour_vertices(contour)
                    shape = self.detect_shape(approx_points)
                    print(shape)
                # cv2.putText(frame, shape, (center_x, center_y),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    def get_signals_by_color(self, frame):
        '''Get the signals in the image'''
        red_mask, green_mask, blue_mask = self.divide_by_color(frame)

        red_contours = self.find_contours(red_mask)
        # green_contours = self.find_contours(green_mask)
        # blue_contours = self.find_contours(blue_mask)

        # out = cv2.bitwise_and(frame, frame, mask=red_mask)
        # display_image(out, "masked")
        # if red_contours:
        #     max_contour = max(red_contours, key=cv2.contourArea)
        #     # print(cv2.contourArea(max_contour))
        #     approx = self.get_contour_vertices(max_contour)
        #     print(len(approx))
        #     # draw_contour(frame, max_contour)
        #     draw_contour_bounding_box(frame, max_contour)

        red_signal_rects = []
        for contour in red_contours:
            area = cv2.contourArea(contour)
            if 60 < area < 7800:
                red_signal_rects.append(cv2.boundingRect(contour))
                # draw_contour_bounding_box(frame, contour)
                # approx_points = self.get_contour_vertices(contour)
                # self.detect_shape(approx_points)

        return red_signal_rects
        # show the images
        # display_image(frame, "Signal finder")
        # display_image(green_mask, "Green mask")
        # display_image(blue_mask, "Blue mask")

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

        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.KERNEL, iterations=1)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self.KERNEL, iterations=1)

        # red_mask = cv2.erode(red_mask, None, iterations=2)
        #red_mask = cv2.dilate(red_mask, None, iterations=1)

        lower = np.array(self.GREEN_BOUNDARY[0], dtype="uint8")
        upper = np.array(self.GREEN_BOUNDARY[1], dtype="uint8")

        green_mask = cv2.inRange(frame, lower, upper)
        # green_mask = cv2.erode(green_mask, None, iterations=2)
        # green_mask = cv2.dilate(green_mask, None, iterations=2)

        lower = np.array(self.BLUE_BOUNDARY[0], dtype="uint8")
        upper = np.array(self.BLUE_BOUNDARY[1], dtype="uint8")

        blue_mask = cv2.inRange(frame, lower, upper)
        # blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, self.KERNEL)
        # blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.KERNEL)

        # blue_mask = cv2.erode(blue_mask, None, iterations=2)
        # blue_mask = cv2.dilate(blue_mask, None, iterations=2)

        # display_image(red_mask, name="Red mask")

        return red_mask, green_mask, blue_mask

    def find_contours(self, mask):
        '''Find the contours of the mask'''
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	                                   cv2.CHAIN_APPROX_SIMPLE)[1]
        #print(hierarchy)
        return contours

    def get_contour_vertices(self, contour):
        '''Get the number of vertices of the contour'''
        # La epsilon controla la precision de la aproximacion del contorno.
        # Ej a mayor epsilon menos puntos y viceversa
        epsilon = 0.01 * cv2.arcLength(contour, True)
        return cv2.approxPolyDP(contour, epsilon, True)

    def detect_shape(self, num_points):
        '''Detect the shape of the contour'''
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        # if the shape is a triangle, it will have 3 vertices
        if len(num_points) == 3:
            shape = "Triangle"

		# if the shape has 4 vertices, it is either a square or
		# a rectangle
        elif len(num_points) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(num_points)
            aspect_ratio = w / float(h)

			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
            shape = "Square" if aspect_ratio >= 0.95 and aspect_ratio <= 1.05 else "Rectangle"

		# if the shape is a pentagon, it will have 5 vertices
        elif len(num_points) == 5:
            shape = "Pentagon"
        elif len(num_points) == 6:
            shape = "Hexagon"
        elif len(num_points) == 8:
            shape = "Stop"
		# otherwise, we assume the shape is a circle
        elif len(num_points) > 8:
            shape = "Posible Circle"
        return shape

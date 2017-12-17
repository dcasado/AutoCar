import math
from statistics import mean, StatisticsError
import time
import cv2
import numpy as np
from multiprocessing.pool import ThreadPool
from utils import display_image

class LineFinder:
    '''Class to find the road lines in a image'''

    def __init__(self):
        '''Line finder constructor'''
        self.pool = ThreadPool(processes=2)

    def best_points_mean(self, group):
        '''Get the best points to use calculating the mean between them'''
        temp_x1l = []
        temp_y1l = []
        temp_x2l = []
        temp_y2l = []
        try:
            for line in group:
                temp_x1, temp_y1, temp_x2, temp_y2 = line["points"]
                temp_x1l.append(temp_x1)
                temp_y1l.append(temp_y1)
                temp_x2l.append(temp_x2)
                temp_y2l.append(temp_y2)
            mean_x1 = mean(temp_x1l)
            mean_y1 = mean(temp_y1l)
            mean_x2 = mean(temp_x2l)
            mean_y2 = mean(temp_y2l)
        except (IndexError, StatisticsError):
            return {"slope": None, "points": []}
        else:
            slope = math.atan2((mean_y2 - mean_y1), (mean_x2 - mean_x1))
            return {"slope": slope,
                    "points": [int(mean_x1), int(mean_y1), int(mean_x2), int(mean_y2)]}

    def best_points_y(self, group):
        '''Get the best points to use taking the ones with the highest and lowest y'''
        best_x1, best_y1, best_x2, best_y2 = None, None, None, None
        try:
            for line in group:
                temp_x1, temp_y1, temp_x2, temp_y2 = line["points"]
                # print("Temp points", temp_x1, temp_y1, temp_x2, temp_y2)
                if best_y1 is None or temp_y1 < best_y1:
                    best_x1 = temp_x1
                    best_y1 = temp_y1
                if best_y2 is None or temp_y2 > best_y2:
                    best_x2 = temp_x2
                    best_y2 = temp_y2
        except IndexError:
            return {"slope": None, "points": []}
        else:
            # print(best_x1, best_y1, best_x2, best_y2)
            if best_x1 is None or best_y1 is None or best_x2 is None or best_y2 is None:
                slope = None
            else:
                slope = math.atan2((best_y2 - best_y1), (best_x2 - best_x1))
            # print("Best points", [best_x1, best_y1, best_x2, best_y2])
            return {"slope": slope,
                    "points": [best_x1, best_y1, best_x2, best_y2]}

    def organize_and_best_group(self, lines):
        '''Merge organize_lines and best_group'''
        groups = []
        temp_group = []
        best_group = []
        # print("\nLines ", lines)
        for i, line in enumerate(lines):
            if not temp_group:
                temp_group.append(line)
            else:
                mean_slope = abs(sum(d["slope"] for d in temp_group) / len(temp_group))
                # print("Mean", mean_slope)
                # print("Line slope", line["slope"])
                if mean_slope * 0.95 < abs(line["slope"]) < mean_slope * 1.05:
                    temp_group.append(line)
                else:
                    groups.append(temp_group)
                    temp_group = []
                    temp_group.append(line)
            if i == len(lines) - 1:
                groups.append(temp_group)
        if groups:
            best_group = max(groups, key=lambda group: len(group))
        # print("\nGroups", groups)
        # print("\nBest group", best_group)
        return best_group

    def merge_lines(self, lines):
        '''Sort and get the best line'''
        # Sort by its slope
        sorted_lines = sorted(lines, key=lambda line: line["slope"])
        # Get the group of lines with more number of lines
        best_group = self.organize_and_best_group(sorted_lines)
        # Merge the lines in only one by the mean or by the y
        final_line = self.best_points_mean(best_group)
        return final_line

    def process_lines(self, lines):
        '''Filter and merge lines to get the best group of lines'''
        if lines is not None:
            positive_lines = []
            negative_lines = []
            atan2 = math.atan2
            for line in lines:
                x_1, y_1, x_2, y_2 = line[0]
                slope = atan2((y_2 - y_1), (x_2 - x_1))
                # print("\nLine before slope", line, " slope", slope)
                # Slope between ~23 and ~80 degrees
                if abs(slope) > 0.4:
                    line_dict = {"slope": slope, "points": line[0]}
                    if slope > 0:
                        positive_lines.append(line_dict)
                    else:
                        negative_lines.append(line_dict)
                    # cv2.line(
                    #     processed_frame, (x_1, y_1), (x_2, y_2), (255, 255, 255), 4)

            # print("Positive lines len:", len(positive_lines),
            #       "Negative lines len: ", len(negative_lines))

            # Calculate final lines asynchronously
            positive_line_async = self.pool.apply_async(self.merge_lines, args=(positive_lines,))
            negative_line_async = self.pool.apply_async(self.merge_lines, args=(negative_lines,))

            positive_line = positive_line_async.get(0.3)
            negative_line = negative_line_async.get(0.3)

            return positive_line, negative_line
        return {"slope": None, "points": []}, {"slope": None, "points": []}

    def get_lines(self, frame):
        '''Process the frame to get the road lanes'''
        processed_frame = frame

        # Create region of interest with width and height of frame
        roi_vertices = self.create_roi_vertices(len(processed_frame[0]), len(processed_frame))
        processed_frame = self.roi(processed_frame, roi_vertices)

        # NOT NECESARY BECAUSE CANNY DOES IT FOR US Change image color to gray
        # Necesary because we do the threshold before
        processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2GRAY)

        ret, processed_frame = cv2.threshold(processed_frame, 130, 255, cv2.THRESH_BINARY)
        processed_frame = cv2.bitwise_not(processed_frame)
        # self.display_image(processed_frame, name="threshold")

        # Calculate edges
        processed_frame = cv2.Canny(processed_frame, 180, 200, apertureSize=3)

        display_image(processed_frame, "canny")

        # Blur image to smooth it
        processed_frame = cv2.GaussianBlur(processed_frame, (3, 3), 0)

        # image, p, rho, threshold to be detected, minimun lenght of line, max gap between lines
        # rho only detects vertical lines with np.pi/180*170
        lines = cv2.HoughLinesP(processed_frame, 1,
                                np.pi / 170, 110, minLineLength=5, maxLineGap=15)

        positive_line, negative_line = self.process_lines(lines)

        return positive_line, negative_line

    def roi(self, image, roi_vertices):
        '''Get the region of interest from the image'''
        # blank mask:
        mask = np.zeros_like(image)
        # fill the mask
        cv2.fillPoly(mask, [roi_vertices], [255, 255, 255])
        # now only show the area that is the mask
        masked = cv2.bitwise_and(image, mask)
        return masked

    def create_roi_vertices(self, width, height):
        '''Create ROI vertices with the image dimensions'''
        return np.array([[0, height],
                         [0, int(height * 0.68)],
                         [int(width * 0.25), int(height * 0.55)],
                         [int(width * 0.75), int(height * 0.55)],
                         [width, int(height * 0.68)],
                         [width, height]])
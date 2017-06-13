import math
import time
import cv2
import numpy as np
from imutils.video import VideoStream, FPS


class Camera:
    '''Camera class'''

    def __init__(self, src=0, width=320, height=240):
        '''Camera constructor'''
        self.camera_width = width
        self.camera_height = height
        self.video_stream = VideoStream(src=src, usePiCamera=True,
                                        resolution=(width, height)).start()
        time.sleep(1.0)

    def close(self):
        '''Close all the resources'''
        self.video_stream.stop()
        cv2.destroyAllWindows()

    def roi(self, image, roi_vertices):
        '''Get the region of interest from the image'''
        # blank mask:
        mask = np.zeros_like(image)
        # fill the mask
        cv2.fillPoly(mask, [roi_vertices], 255)
        # now only show the area that is the mask
        masked = cv2.bitwise_and(image, mask)
        return masked

    def merge_lines(self, processed_frame, lines, draw_lines):
        '''Filter and merge lines to get the best group of lines'''
        if lines is not None:
            positive_lines = []
            negative_lines = []
            atan = math.atan
            for line in lines:
                x_1, y_1, x_2, y_2 = line[0]
                slope = atan((y_2 - y_1) / (x_2 - x_1))
                # Slope between ~23 and ~80 degrees
                if 1.4 > abs(slope) > 0.4:
                    line_dict = {"slope": slope, "points": line[0]}
                    if slope > 0:
                        positive_lines.append(line_dict)
                    else:
                        negative_lines.append(line_dict)
            # print("Positive lines len:", len(positive_lines),
            #       "Negative lines len: ", len(negative_lines))

            # Order the lines by its slope
            positive_lines = sorted(
                positive_lines, key=lambda line: line["slope"])
            negative_lines = sorted(
                negative_lines, key=lambda line: line["slope"])
            # positive_group_lines = self.organize_lines(positive_lines)
            # negative_group_lines = self.organize_lines(negative_lines)
            # best_positive_group = self.choose_best_group(positive_group_lines)
            # best_negative_group =
            # self.choose_best_group(negative_group_lines)

            best_positive_group = self.organize_and_best_group(positive_lines)
            best_negative_group = self.organize_and_best_group(negative_lines)

            # print("All groups", group_lines)
            # print("Best positive group", best_positive_group)
            # print("Best positive group2", best_positive_group2)

            # print("Best positive group", best_positive_group)
            # print("Best negative group", best_negative_group)

            positive_point = self.best_points2(best_positive_group)
            negative_point = self.best_points2(best_negative_group)

            # print("Positive points", positive_points)
            # print("Negative points", negative_points)

            # Draw the lines in the frame
            if draw_lines:
                try:
                    cv2.line(
                        processed_frame, (positive_point["points"]
                                          [0], positive_point["points"][1]),
                        (positive_point["points"][2],
                         positive_point["points"][3]), (255, 255, 255), 4)
                    cv2.line(
                        processed_frame, (negative_point["points"]
                                          [0], negative_point["points"][1]),
                        (negative_point["points"][2],
                         negative_point["points"][3]), (255, 255, 255), 4)
                except TypeError:
                    pass

            return positive_point, negative_point
        return {"slope": -1, "points": [-1, -1, -1, -1]}, {"slope": -1, "points": [-1, -1, -1, -1]}

    def best_points2(self, group):
        '''Get the best points to use'''
        best_x1, best_y1, best_x2, best_y2 = None, None, None, None
        try:
            if group[0]["slope"] > 0:
                for line in group:
                    temp_x1, temp_y1, temp_x2, temp_y2 = line["points"]
                    # print("Temp points", temp_x1, temp_y1, temp_x2, temp_y2)
                    if best_y1 is None or temp_y1 > best_y1:
                        best_x1 = temp_x1
                        best_y1 = temp_y1
                    if best_y2 is None or temp_y2 < best_y2:
                        best_x2 = temp_x2
                        best_y2 = temp_y2
            else:
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
            return {"slope": -1,
                    "points": [best_x1, best_y1, best_x2, best_y2]}
        else:
            # print(best_x1, best_y1, best_x2, best_y2)
            slope = math.atan((best_y2 - best_y1) / (best_x2 - best_x1))
            return {"slope": slope,
                    "points": [best_x1, best_y1, best_x2, best_y2]}

    def best_points(self, group):
        '''Get the best points to use'''
        best_x1, best_y1, best_x2, best_y2 = -1, -1, -1, -1
        if group:
            if group[0]["slope"] > 0:
                best_x1, best_y1 = -1, -1
                best_x2, best_y2 = self.camera_width + 1, self.camera_height + 1
                for line in group:
                    temp_x1, temp_y1, temp_x2, temp_y2 = line["points"]
                    # print("Temp points", temp_x1, temp_y1, temp_x2, temp_y2)
                    if temp_y1 > best_y1:
                        best_x1 = temp_x1
                        best_y1 = temp_y1
                    if temp_y2 < best_y2:
                        best_x2 = temp_x2
                        best_y2 = temp_y2
            else:
                best_x1, best_y1 = self.camera_width + 1, self.camera_height + 1
                best_x2, best_y2 = -1, -1
                for line in group:
                    temp_x1, temp_y1, temp_x2, temp_y2 = line["points"]
                    # print("Temp points", temp_x1, temp_y1, temp_x2, temp_y2)
                    if temp_y1 < best_y1:
                        best_x1 = temp_x1
                        best_y1 = temp_y1
                    if temp_y2 > best_y2:
                        best_x2 = temp_x2
                        best_y2 = temp_y2

        # print(best_x1, best_y1, best_x2, best_y2)
        if best_x1 == -1 or best_x2 == -1 or best_y1 == -1 or best_y2 == -1:
            slope = -1
        else:
            slope = math.atan((best_y2 - best_y1) / (best_x2 - best_x1))
        return {"slope": slope,
                "points": [best_x1, best_y1, best_x2, best_y2]}

    def organize_and_best_group(self, lines):
        '''Merge organize_lines and best_group'''
        def compare_group(group1, group2):
            '''Compare the lenght of the group and return the one with greater lenght'''
            if len(group1) > len(group2):
                return group1
            else:
                return group2
        best_group = []
        temp_group = []
        for line in lines:
            if not temp_group:
                temp_group.append(line)
                best_group = compare_group(best_group, temp_group)
            else:
                mean_slope = float(sum(
                    d["slope"] for d in temp_group)) / len(temp_group)
                # print("Mean", mean_slope)
                if mean_slope * 0.95 < line["slope"] < mean_slope * 1.05:
                    temp_group.append(line)
                    best_group = compare_group(best_group, temp_group)
                else:
                    best_group = compare_group(best_group, temp_group)
                    temp_group = []
                    temp_group.append(line)
        return best_group

    def process_frame(self, frame, draw_lines):
        '''Process the frame to get the road lanes'''
        processed_frame = frame

        # NOT NECESARY BECAUSE CANNY DOES IT FOR US Change image color to gray
        # processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate edges
        processed_frame = cv2.Canny(processed_frame, 100, 200, apertureSize=3)
        # processed_frame = auto_canny(processed_frame)

        # Blur image to smooth it
        processed_frame = cv2.GaussianBlur(processed_frame, (5, 5), 0)

        # image, p, rho, threshold to be detected, minimun lenght of line, max gap between lines
        # rho only detects vertical lines with np.pi/180*170
        lines = cv2.HoughLinesP(processed_frame, 1, np.pi /
                                180, 110, minLineLength=20, maxLineGap=10)
        positive_line, negative_line = self.merge_lines(
            processed_frame, lines, draw_lines)
        # self.draw_lines(processed_frame, lines)
        return processed_frame, positive_line, negative_line

    def get_processed_frame(self, roi_vertices, display_image):
        '''Get the processed frame'''
        frame = self.video_stream.read()

        # Rotate frame 180 degrees
        frame = np.rot90(frame, 2)

        roi_frame = self.roi(frame, roi_vertices)
        final_frame, final_positive_line, final_negative_line = self.process_frame(
            roi_frame, display_image)

        if display_image:
            self.display_image(final_frame, "Final")

        return final_positive_line, final_negative_line

    def display_image(self, image, name="frame"):
        ''' Display the image '''
        cv2.imshow(name, image)


if __name__ == "__main__":
    camera = Camera()
    VERTICES = np.array([[0, 240], [0, 140], [100, 100],
                         [220, 100], [320, 140], [320, 240]])
    start_time = time.time()
    fps = FPS().start()
    while fps._numFrames < 200:
        # while True:
        POSITIVE_LINE, NEGATIVE_LINE = camera.get_processed_frame(
            VERTICES, True)

        # print("Positive line:", positive_line)
        # print("Negative line:", negative_line)

        if POSITIVE_LINE["slope"] != -1 and NEGATIVE_LINE["slope"] != -1:
            print("Go straight")
        elif POSITIVE_LINE["slope"] == -1 and NEGATIVE_LINE["slope"] != -1:
            print("Turn right")
        elif POSITIVE_LINE["slope"] != -1 and NEGATIVE_LINE["slope"] == -1:
            print("Turn left")
        else:
            print("Go anywhere")
        print("Frame took ", time.time() - start_time)
        start_time = time.time()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        fps.update()
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    camera.close()

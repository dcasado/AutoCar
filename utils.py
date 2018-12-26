from cv2 import imshow, line as cv2_line, drawContours, rectangle, boundingRect

SHOW_IMAGES = True

def display_image(image, name="frame"):
    ''' Display the image '''
    if SHOW_IMAGES:
        imshow(name, image)

def draw_line(processed_frame, line):
    '''Draw the lines in the frame'''
    # Draw line
    if SHOW_IMAGES:
        try:
            cv2_line(processed_frame,
                     (line["points"][0],
                      line["points"][1]),
                     (line["points"][2],
                      line["points"][3]),
                     (255, 255, 255), 4)
        except (TypeError, IndexError):
            pass

def draw_contour(frame, contour):
    '''Draw contours'''
    if SHOW_IMAGES:
        drawContours(frame, [contour], 0, (0, 255, 0), 2)

def draw_contour_bounding_box(frame, contour):
    '''Draw bounding box of contour'''
    if SHOW_IMAGES:
        x_rect, y_rect, w_rect, h_rect = boundingRect(contour)
        rectangle(frame, (x_rect, y_rect), (x_rect + w_rect, y_rect + h_rect), (0, 255, 0), 2)

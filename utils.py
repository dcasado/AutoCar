import cv2

def display_image(image, name="frame"):
    ''' Display the image '''
    cv2.imshow(name, image)

def draw_line(processed_frame, line):
    '''Draw the lines in the frame'''
    # Draw line
    try:
        cv2.line(processed_frame,
                 (line["points"][0],
                  line["points"][1]),
                 (line["points"][2],
                  line["points"][3]),
                 (255, 255, 255), 4)
    except (TypeError, IndexError):
        pass

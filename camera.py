import time
import cv2
import numpy as np
from imutils.video import VideoStream, FPS


class Camera:
    '''Camera class'''

    def __init__(self, src=0, width=320, height=240):
        '''Camera constructor'''
        self.video_stream = VideoStream(src=src, usePiCamera=True,
                                        resolution=(width, height)).start()
        time.sleep(0.5)

    def close(self):
        '''Close all the resources'''
        self.video_stream.stop()
        cv2.destroyAllWindows()

    def get_frame(self):
        '''Get the processed frame'''
        frame = self.video_stream.read()
        # frame = cv2.imread("muo-linux-raspbian-pixel-desktop.png", cv2.IMREAD_COLOR)
        # Rotate frame 180 degrees
        frame = np.rot90(frame, 2).copy()
        return frame


if __name__ == "__main__":
    camera = Camera()
    start_time = time.time()
    fps = FPS().start()
    while fps._numFrames < 200:
        # while True:
        FRAME = camera.get_frame()
        # print("Positive line:", positive_line)
        # print("Negative line:", negative_line)
        print("Frame took ", time.time() - start_time)
        start_time = time.time()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        fps.update()
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    camera.close()

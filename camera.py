# Modified from https://github.com/jrosebr1/imutils/blob/master/imutils/video/pivideostream.py with MIT License

from threading import Thread
from picamera import PiCamera
from picamera.array import PiRGBArray

class Camera:
    '''Camera class'''

    def __init__(self, width=320, height=240, framerate=32, hflip=False, vflip=False):
        '''Camera constructor'''
        self.camera = PiCamera()
        self.camera.resolution = (width, height)
        self.camera.framerate = framerate
        self.raw_capture = PiRGBArray(self.camera, size=(width, height))
        self.stream = self.camera.capture_continuous(self.raw_capture,
                                                     format="bgr", use_video_port=True)
        self.camera.hflip = hflip
        self.camera.vflip = vflip
        # initialize the frame and the variable used to indicate
		# if the thread should be stopped
        self.frame = None
        self.stopped = False

    def start(self):
        '''start the thread to read frames from the video stream'''
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        '''keep looping infinitely until the thread is stopped'''
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.raw_capture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.raw_capture.close()
                self.camera.close()
                return

    def read(self):
        '''return the frame most recently read'''
        return self.frame

    def stop(self):
        '''indicate that the thread should be stopped'''
        self.stopped = True

import numpy as np
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
import picamera
import picamera.array

def vision_loop(self):
        """
        Computes the path from most recent frame using computer vision pipeline
        """
        go = True
        nframe = 100
        i = 0
        
        dim = (480, 720, 3)
        frame = np.empty(shape=dim, dtype=np.uint8)
        with picamera.PiCamera(framerate=60, resolution=(720,480)) as camera:
            while go:
                #Capture image
                camera.capture(frame, format='bgr', use_video_port=True)
                frame[0:dim[0]][(dim[1]//2 - 10):(dim[1]//2 + 10)] = (0, 0, 255)
                cv2.imshow('frame', frame)  
                cv2.waitKey(1) & 0xFF
                i = i + 1
                if i > nframe:
                    break


vision_loop()    
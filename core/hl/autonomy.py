import numpy as np
import cv2
import numpy as np
import matplotlib.pyplot as plt
import picamera
import picamera.array
import lanedetection as ld
import stateestimation as se
import purepursuit as pp

#Description: The high level program which unites computer functions
class Autonomy:
    """
    The digit model of vehicle autnomy
    """

    def __init__(self, sm):
        """
        Default constructor

        param sm: The speed matrix containing speeds and their associated distances or times
        """

        #----------Timing Data-----------#
        self.time = 0

        #---------Resource Management----------#
        #The frequency of the autonomy loop
        self.afreq = 500
        
        #Whether or not the system has produced new computer vision data
        self.newvis = False

        #---------Hardware Data--------#
        self.camera = None

        self.teensy = None
        
        #----------Planning Data----------#
        #The speed matrix containing speeds and their associated distances or times
        self.sm = sm

        #----------Sensor Data-----------#
        #The latest frame from the camera
        self.ltst_frame = None

        #The latest tstate from the teensy
        self.ltst_tstate = None

        #----------Subsystem Data----------#
        self.lane = None

        self.teensy_state = None

        self.state = None

        self.str_angle = None

        #----------Control Data----------#
        

    def read_tstate(self):
        """
        Gets the most recent measurements from the teensy
        """

        
        
        

    def capture_frame(self):
        """
        Computes the path from most recent frame using computer vision pipeline
        """
        
        dim = (480, 720, 3)
        frame = np.empty(shape=dim, dtype=np.uint8)
        with picamera.PiCamera(framerate=60, resolution=(720,480)) as camera:
            #Capture image
            camera.capture(frame, format='bgr', use_video_port=True)
            #Store the image in the latest frame location

    def vision(self):
        print()
                       
    def autonomy(self):
        """
        Computes the state from the most recent sensor input data, passes the state to the pure pursuit and pi control algorithms and outputs the resulting control signals
        """

        state = se.pipeline()

        if self.newvis == True:
            #Update the state using the new vision case
            self.state = se.pipeline(self.lane.color_pwarped, self.lane.path_fit)

            #Indicate that vision data has been used
            self.newvis = False

        else:
            #Update the state using the dead reckoning case
            self.state = se.pipeline(self)

        #Run the pure pursuit pipeline
        pursuit = pp.pipeline(state)

    

    


    

    





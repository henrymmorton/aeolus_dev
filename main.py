import cv2
import numpy as np
import matplotlib.pyplot as plt
from core.cv.pipeline_dev.lanedetection import lane_detection_pipeline
from core.se.stateestimation import VehicleState, state_est_pipeline
from core.sc.purepursuit import Pursuit, pure_pursuit_pipeline
from core.vhcl.vehicle import Vehicle



#Load the image
test_image = cv2.imread('test/test_wet.jpeg')

# Establish the vehicle constants
V_LENGTH = 0.4
V_WIDTH = 0.15
WHEEL_RADIUS = 0.05
A2A_LENGTH = 0.2
CAM_HEIGHT = 0.1
CA_OFFSET = 0.1
CAM2IM = 0.1
my_vehicle = Vehicle(V_LENGTH, V_WIDTH, WHEEL_RADIUS, A2A_LENGTH, CAM_HEIGHT, CA_OFFSET, CAM2IM)


#Run the lane detection pipeline
lane = lane_detection_pipeline(test_image, plotIt = False, myThresholds = ((190, 255), (150, 255), (150, 255), (210 ,255)))

#Run the state estimation pipeline
state = state_est_pipeline(my_vehicle, lane.color_pwarped, lane.path_fit)

breakpoint()

#Run the pure pursuit pipeline
pursuit = pure_pursuit_pipeline(state)

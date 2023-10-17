import cv2
import numpy as np
import matplotlib.pyplot as plt
import lanedetection as ld
import stateestimation as se
import purepursuit as pp

#Load the image
test_image = cv2.imread('test_wet.jpeg')

#Run the lane detection pipeline
lane = ld.pipeline(test_image, plotIt = False, myThresholds = ((190, 255), (150, 255), (150, 255), (210 ,255)))

#Run the state estimation pipeline
state = se.pipeline(lane.color_pwarped, lane.path_fit)

#Run the pure pursuit pipeline
pursuit = pp.pipeline(state)

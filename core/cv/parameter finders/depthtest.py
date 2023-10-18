import cv2
import lanedetection as ld

#Load the image
file = input("choose file")
test_image = cv2.imread(file)

#Create a instance of Lane
depthlane = ld.Lane(original=test_image, roi_corners=[(742,452), (0,969), (1920, 969), (1178, 452)])

birdseye = depthlane.perspective_transform(img=test_image, plot=True)
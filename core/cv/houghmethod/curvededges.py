import cv2
import numpy as np

img =cv2.imread('strack.jpeg')
height,width=img.shape[:2]
sclimg=cv2.resize(img,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_AREA)

cv2.startWindowThread()

cv2.imshow('straight track',sclimg)
cv2.waitKey(0)
cv2.destroyAllWindows()

blr = cv2.medianBlur(sclimg, 5)
cv2.imshow('blurred',blr)
cv2.waitKey(0)
cv2.destroyAllWindows()

hls = cv2.cvtColor(blr, cv2.COLOR_BGR2HLS)
lower_white=np.array([0,140 ,0])
upper_white=np.array([7,255,255])
mask = cv2.inRange(hls,lower_white,upper_white)
masked = cv2.bitwise_and(hls, hls, mask = mask)
maskedrbg = cv2.cvtColor(masked, cv2.COLOR_HLS2BGR)
cv2.imshow('white channel',maskedrbg)
cv2.waitKey(0)
cv2.destroyAllWindows()

edges = cv2.Canny(mask,50,100,apertureSize=3)
cv2.imshow('straight track edges',edges)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.waitKey(1)

lanes = cv2.HoughLinesP(edges, 1, (1*np.pi)/180, 100, 300, 30, 20)

if lanes is not None:
    for i in range(0,len(lanes)):
        l = lanes[i][0]
        cv2.line(sclimg, (l[0],l[1]), (l[2], l[3]), (0,255,0), 2)
        

cv2.imshow('straight track lines',sclimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.waitKey(1)
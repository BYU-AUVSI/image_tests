# Standard imports
import cv2
import numpy as np
 
# Read image
im = cv2.imread("2017competition/all_images/061517_11-38-17-269.jpg", cv2.IMREAD_GRAYSCALE)

#Thresholding is nice when the targets are light colored, but they won't always be
# retval, threshold = cv2.threshold(im, 103, 255, cv2.THRESH_BINARY_INV)
# cv2.imshow("Threshold", threshold)
# cv2.waitKey(0)

# Set params
params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 0
params.maxThreshold = 255
params.thresholdStep = 25

params.filterByColor = False
params.filterByArea = True
params.filterByCircularity = False
params.filterByInertia = True
params.filterByConvexity = False
params.minArea = 75

params.minInertiaRatio = .1

print(str(params.minThreshold) + " " + str(params.maxThreshold) + " " + str(params.thresholdStep))
print params.filterByColor
print params.filterByArea
print params.filterByCircularity
print params.filterByInertia
print params.filterByConvexity
print params.minArea
print params.minInertiaRatio
print params.maxInertiaRatio

# Set up the detector with default parameters.
detector = cv2.SimpleBlobDetector_create(params)
 
# Detect blobs.
keypoints = detector.detect(im)
print(len(keypoints))
 
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)

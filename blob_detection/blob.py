import cv2
import numpy as np
import sys
import os
import glob
import argparse

DEFAULT_IMAGE_PATH = "2017competition/all_images"
FOUND_TARGETS_PATH = "./found_targets"
NO_FOUND_TARGETS_PATH = "./no_found_targets"

def processImage(filename, interactive):
    # Read image
    im_color = cv2.imread(filename, cv2.IMREAD_COLOR)
    im = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)

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

    # print(str(params.minThreshold) + " " + str(params.maxThreshold) + " " + str(params.thresholdStep))
    # print params.filterByColor
    # print params.filterByArea
    # print params.filterByCircularity
    # print params.filterByInertia
    # print params.filterByConvexity
    # print params.minArea
    # print params.minInertiaRatio
    # print params.maxInertiaRatio

    # Set up the detector with default parameters.
    detector = cv2.SimpleBlobDetector_create(params)
    
    # Detect blobs.
    keypoints = detector.detect(im)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(im_color, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    if len(keypoints) > 0:
        cv2.imwrite(os.path.join(FOUND_TARGETS_PATH, os.path.basename(filename)), im_with_keypoints)
    else:
        cv2.imwrite(os.path.join(NO_FOUND_TARGETS_PATH, os.path.basename(filename)), im_color)
    
    # Show keypoints
    if interactive:
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--interactive", help="Show each image as it is processed", action='store_true')
    parser.add_argument("-image_path", help="Filepath to look for *.jpg images")
    args = parser.parse_args()

    # Get images for classifying
    image_path = args.image_path if args.image_path else DEFAULT_IMAGE_PATH

    if not os.path.exists(FOUND_TARGETS_PATH):
        os.makedirs(FOUND_TARGETS_PATH)
    if not os.path.exists(NO_FOUND_TARGETS_PATH):
        os.makedirs(NO_FOUND_TARGETS_PATH)

    for filename in glob.glob(os.path.join(image_path, "*.jpg")):
        processImage(filename, args.interactive)

if __name__ == "__main__":
    main()

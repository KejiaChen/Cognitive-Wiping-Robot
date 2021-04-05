## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import random
from Motion_Panning_Algorithms.NearstNeighbors import nearst_neighbor_planning as nn

color_frame = cv2.imread("/home/kejia/Cognitive-Wiping-Robot/ipad_input.jpg")

color_image = np.asanyarray(color_frame)
color_colormap_dim = color_image.shape
color_image = cv2.resize(color_image, dsize=(int(color_colormap_dim[1]/2), int(color_colormap_dim[0]/2)), interpolation=cv2.INTER_AREA)

# select region of interest
roi = cv2.selectROI(windowName="roi", img=color_image, showCrosshair=True, fromCenter=False)
x, y, w, h = roi
cv2.rectangle(img=color_image, pt1=(x, y), pt2=(x + w, y + h), color=(0, 0, 255), thickness=2)
color_image = color_image[y:y + h, x:x + w]

# cv2.imshow('img', color_image)
# cv2.waitKey(0)

gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

ret, thresh = cv2.threshold(gray_img, 127, 255, 0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# print("length of contours", len(contours))

color_image_dim = color_image.shape

if contours:
    # cv2.drawContours(gray_img, contours, -1, (0, 255, 0), 3)
    # plot largest contour
    max_cnt = np.array(sorted(contours, key=cv2.contourArea)[-1])
    # cv2.drawContours(color_image, max_cnt, -1, (0, 0, 255), 3)

    # Optional: plot rectangular contour
    # rect = cv2.minAreaRect(max_cnt)
    # box = cv2.boxPoints(rect)
    # box = np.int0(box)
    # gray_img = cv2.drawContours(gray_img, [box], 0, (0, 0, 255), 2)

    # x, y, w, h = cv2.boundingRect(max_cnt)
    # color_image = cv2.rectangle(color_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # TODO: plot largest contour on white color

    # only work on the contours inside max_cnt
    temp_contours = contours[:]
    for i in range(len(temp_contours)-1, -1, -1):
        cnt = temp_contours[i]
        for j in range(len(cnt)):
            point = cnt[j]
            if cv2.pointPolygonTest(max_cnt, (point[0][0], point[0][1]), False) == -1.0:
                contours.pop(i)
                break

    draw_contours = contours[:]  # contours are stains to be cleaned
    # draw_contours.append(max_cnt)

    nodes = []
    # start position
    start_x = random.randint(0, color_image_dim[0])
    start_y = random.randint(0, color_image_dim[1])
    nodes.append(np.array([start_x, start_y]))
    color_image = cv2.circle(color_image, (start_x, start_y), radius=0, color=(255, 0, 0), thickness=7)

    # stain centers as nodes
    for cnt in contours:
        M = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        nodes.append(np.array([cx, cy]))
        color_image = cv2.circle(color_image, (cx, cy), radius=0, color=(0, 0, 255), thickness=7)

    for u in nodes:
        print("node: ", u)

    # motion planning
    path = nn(nodes)

    # plot path
    for k in range(len(path)-1):
        s = path[k]
        e = path[k+1]
        color_image = cv2.line(color_image, (s[0], s[1]),  (e[0], e[1]), color=(0, 0, 255), thickness=2)

    # Show images
    cv2.drawContours(color_image, draw_contours, -1, (0, 255, 0), 3)
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    # cv2.imshow('RealSense', red_image)
    cv2.waitKey(0)

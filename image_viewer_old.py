## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import random
from sklearn.neighbors import RadiusNeighborsClassifier
from skimage.draw import disk

from Motion_Panning_Algorithms.Planner import Planner

real_max = 500

color_frame = cv2.imread("/home/kejia/Cognitive-Wiping-Robot/static_image_input/ipad_input_words.jpg")

color_image = np.asanyarray(color_frame)
color_colormap_dim = color_image.shape
color_image = cv2.resize(color_image, dsize=(int(color_colormap_dim[1]/2), int(color_colormap_dim[0]/2)), interpolation=cv2.INTER_AREA)
original_image = color_image

# select region of interest
roi = cv2.selectROI(windowName="roi", img=color_image, showCrosshair=True, fromCenter=False)
x, y, w, h = roi
cv2.rectangle(img=color_image, pt1=(x, y), pt2=(x + w, y + h), color=(0, 0, 255), thickness=2)
color_image = color_image[y:y + h, x:x + w]

# sampling
# color_image = cv2.resize(color_image, dsize=(int(color_colormap_dim[1]/4), int(color_colormap_dim[0]/4)), interpolation=cv2.INTER_AREA)

# cv2.imshow('img', color_image)
# cv2.waitKey(0)

gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
temp_gray_img = gray_img[:]

gray_img_dim = gray_img.shape
# TODO: hsv

ret, thresh = cv2.threshold(gray_img, 127, 255, cv2.THRESH_BINARY_INV)
# ret, thresh = cv2.threshold(gray_img, 127, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

color_image_dim = color_image.shape

black_points = np.nonzero(thresh)

# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
# cv2.imshow('RealSense', thresh)
# cv2.waitKey(0)

# Specify structure shape and kernel size.
# Kernel size increases or decreases the area
# of the rectangle to be detected.
rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
#
# # Appplying dilation on the threshold image
dilation = cv2.dilate(thresh, rect_kernel, iterations=1)

contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# print("length of contours", len(contours))

# # Preprocessing for GTSP
# mask = np.zeros_like(gray_img)
# radius = 3
# points = np.nonzero(gray_img)
# num = points[0].size
# node = [points[0][0], points[1][0]]
# points_set = []
# for i in range(1, num):
#     test_node = points[i][i]
#     rr, cc = disk(tuple(node), radius)
#     mask[rr, cc] = 1
#     # mask[]
#     # if np.linalg.norm(test_node, node)

if contours:
    # cv2.drawContours(gray_img, contours, -1, (0, 255, 0), 3)
    # Default: bounding contour
    contours = sorted(contours, key=cv2.contourArea)
    max_cnt = np.array(sorted(contours, key=cv2.contourArea)[-1])
    # cv2.drawContours(color_image, max_cnt, -1, (0, 0, 255), 3)

    # Optional: plot rectangular contour
    # rect = cv2.minAreaRect(max_cnt)
    # box = cv2.boxPoints(rect)
    # box = np.int0(box)
    # gray_img = cv2.drawContours(gray_img, [box], 0, (0, 0, 255), 2)

    # box_list = []
    # for cnt in contours:
    #     # Option 1.1: plot bounding rectangles
    #     x, y, w, h = cv2.boundingRect(cnt)
    #     color_image = cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # Option 1.2: plot rotated rectangles
    # rect = cv2.minAreaRect(cnt)
    # box = cv2.boxPoints(rect)
    # box = np.int0(box)
    # print("box:", box)
    # for j in range(len(box)):
    #     point = box[j]
    #     if cv2.pointPolygonTest(max_cnt, (point[0], point[1]), False) == 1.0:
    #         box_list.append(box)
    #         color_image = cv2.drawContours(color_image, [box], 0, (0, 0, 255), 2)
    # x, y, w, h = cv2.boundingRect(cnt)
    # color_image = cv2.rectangle(color_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # When the stains are not dense (e.g. words), the box_list contains many repeated
    # and unnecessary boxes

    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', color_image)
    # cv2.waitKey(0)

    # only work on the contours/rectangles inside max_cnt
    temp_contours = contours[:]
    for i in range(len(temp_contours)-1, -1, -1):
        cnt = temp_contours[i]
        for j in range(len(cnt)):
            point = cnt[j]
            if cv2.pointPolygonTest(max_cnt, (point[0][0], point[0][1]), False) == -1.0:
                contours.pop(i)
                break

    draw_contours = contours[:]  # contours are stains to be cleaned
    contours.pop(-1)
    contours.pop(-1)

    # cv2.drawContours(color_image, draw_contours, -1, (0, 255, 0), 3)
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', color_image)
    # # cv2.imshow('RealSense', red_image)
    # cv2.waitKey(0)
    # # # draw_contours.append(max_cnt)

    nodes = []
    # start position
    # start_x = random.randint(0, color_image_dim[0])
    # start_y = random.randint(0, color_image_dim[1])
    start_x = 225
    start_y = 195
    nodes.append(np.array([start_x, start_y]))
    color_image = cv2.circle(color_image, (start_x, start_y), radius=0, color=(255, 0, 0), thickness=7)

    # stain centers as nodes
    radius = 10
    shape = gray_img.shape
    mask = np.zeros_like(gray_img)
    mask_image_1 = color_image.copy()
    mask_image_2 = color_image.copy()
    for cnt in contours:
        M = cv2.moments(cnt)
        if M['m00']:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            if mask[cx, cy] == 0:
                rr, cc = disk((cx, cy), radius, shape=shape)
                mask[rr, cc] = 1
                mask_image_1 = cv2.circle(mask_image_1, (cx, cy), radius=radius, color=(0, 255, 0),
                                          thickness=-1)
                nodes.append(np.array([cx, cy]))
                color_image = cv2.circle(color_image, (cx, cy), radius=0, color=(0, 0, 255), thickness=3)

            for j in range(len(cnt)):
                point = cnt[j]
                # center_x = cx
                # center_y = cy
                while mask[point[0][1], point[0][0]] == 0:
                    # center_x = int((point[0][0] + center_x) / 2)
                    # center_y = int((point[0][1] + center_y) / 2)
                    # rr, cc = disk((center_x, center_y), radius)
                    rr, cc = disk((point[0][1], point[0][0]), radius, shape=shape)
                    mask[rr, cc] = 1
                    mask_image_1 = cv2.circle(mask_image_1, (point[0][0], point[0][1]), radius=radius, color=(0, 255, 0), thickness=-1)
                    nodes.append(np.array([point[0][0], point[0][1]]))
                    color_image = cv2.circle(color_image, (point[0][0], point[0][1]), radius=0, color=(0, 0, 255), thickness=3)

    # for p in range(black_points[0].size):
    #     if mask[black_points[0][p], black_points[0][1]] == 0:
    #     for j in range(len(cnt)):
    #         point = cnt[j]
    #         center_x = cx
    #         center_y = cy
    #         while mask[point[0][1], point[0][0]] == 0:
    #             center_x = int((point[0][0] + center_x) / 2)
    #             center_y = int((point[0][1] + center_y) / 2)
    #             rr, cc = disk((center_x, center_y), radius)
    #             mask[rr, cc] = 1
    #             nodes.append(np.array([center_x, center_y]))
    #             color_image = cv2.circle(color_image, (center_x, center_y), radius=0, color=(0, 0, 255), thickness=2)

    for u in nodes:
        print("node: ", u)

    # alpha = 0.3
    # cover_img = cv2.addWeighted(mask_image_1, alpha, color_image, 1 - alpha, 0)
    # #
    # # cv2.drawContours(color_image, draw_contours, -1, (0, 255, 0), 1)
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', cover_img)
    # # cv2.imshow('RealSense', red_image)
    # cv2.waitKey(0)

    # motion planning
    motion_planner = Planner(nodes, k=15, r=50)
    # plot cost graph
    G = motion_planner.get_cost_graph()

    for n, nbrs in G.adj.items():
        for nbr, eattr in nbrs.items():
            wt = eattr['weight']
            node = nodes[n]
            adj_node = nodes[nbr]
            # color_image = cv2.line(color_image, (node[0], node[1]), (adj_node[0], adj_node[1]), color=(0, 255, 0),thickness=1)

    # alpha = 0.3
    # cover_img = cv2.addWeighted(mask_image_1, alpha, color_image, 1 - alpha, 0)

    # Show images
    # cv2.drawContours(color_image, draw_contours, -1, (0, 255, 0), 3)
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', color_image)
    # # cv2.imshow('RealSense', red_image)
    # cv2.waitKey(0)

    path, travelled_dst = motion_planner.ant_colony()

    print("path planned")

    # plot path
    for k in range(len(path) - 1):
        s = path[k]
        e = path[k + 1]
        # mask_image = cv2.circle(mask_image, (point[0][0], point[0][1]), radius=radius, color=(0, 255, 0), thickness=-1)
        mask_image_2 = cv2.line(mask_image_2, (s[0], s[1]), (e[0], e[1]), color=(255, 0, 0), thickness=radius+5)

    alpha = 0.3
    # mask_image =
    # cover_img = cv2.addWeighted(mask_image_1, alpha, color_image, 1 - alpha, 0)
    cover_img = cv2.addWeighted(mask_image_2, alpha, color_image, 1 - alpha, 0)

    # Show images
    # cv2.drawContours(color_image, draw_contours, -1, (0, 255, 0), 3)
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', cover_img)
    # cv2.imshow('RealSense', red_image)
    cv2.waitKey(0)




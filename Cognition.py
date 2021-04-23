## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import copy
import random
from skimage.draw import disk
from Motion_Panning_Algorithms.Planner import Planner


class Cognition():
    def __init__(self,
                 image,
                 streaming=0,
                 radius=10,
                 planner_k=15,
                 planner_r=50):
        self.stream = streaming
        self.img = image
        color_image_dim = self.img.shape
        self.gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        temp_gray_img = self.gray_img[:]
        gray_img_dim = self.gray_img.shape
        # TODO: hsv

        self.radius = radius

        ret, thresh = cv2.threshold(self.gray_img, 127, 255, cv2.THRESH_BINARY_INV)
        # ret, thresh = cv2.threshold(gray_img, 127, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        black_points = np.nonzero(thresh)

        # Specify structure shape and kernel size.
        # Kernel size increases or decreases the area
        # of the rectangle to be detected.
        rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # Appplying dilation on the threshold image
        dilation = cv2.dilate(thresh, rect_kernel, iterations=1)

        self.contours, self.hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # planner
        self.pk = planner_k
        self.pr = planner_r

    def list_nodes(self):
        node_list = []
        # start position
        # start_x = random.randint(0, color_image_dim[0])
        # start_y = random.randint(0, color_image_dim[1])
        start_x = 225
        start_y = 195
        node_list.append(np.array([start_x, start_y]))
        self.img = cv2.circle(self.img, (start_x, start_y), radius=0, color=(255, 0, 0), thickness=7)

        # stain centers as nodes
        # radius = 10
        shape = self.gray_img.shape
        mask = np.zeros_like(self.gray_img)
        mask_image_1 = self.img.copy()
        for cnt in self.contours:
            M = cv2.moments(cnt)
            if M['m00']:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                if mask[cy, cx] == 0:
                    rr, cc = disk((cy, cx), self.radius, shape=shape)
                    mask[rr, cc] = 1
                    mask_image_1 = cv2.circle(mask_image_1, (cx, cy), radius=self.radius, color=(0, 255, 0),
                                              thickness=-1)
                    node_list.append(np.array([cx, cy]))
                    self.img = cv2.circle(self.img, (cx, cy), radius=0, color=(0, 0, 255), thickness=3)

                for j in range(len(cnt)):
                    point = cnt[j]
                    # center_x = cx
                    # center_y = cy
                    while mask[point[0][1], point[0][0]] == 0:
                        # center_x = int((point[0][0] + center_x) / 2)
                        # center_y = int((point[0][1] + center_y) / 2)
                        # rr, cc = disk((center_x, center_y), radius)
                        rr, cc = disk((point[0][1], point[0][0]), self.radius, shape=shape)
                        mask[rr, cc] = 1
                        mask_image_1 = cv2.circle(mask_image_1, (point[0][0], point[0][1]), radius=self.radius,
                                                  color=(0, 255, 0), thickness=-1)
                        node_list.append(np.array([point[0][0], point[0][1]]))
                        self.img = cv2.circle(self.img, (point[0][0], point[0][1]), radius=0, color=(0, 0, 255),
                                                 thickness=3)

        # for u in node_list:
        #     print("node: ", u)

        # alpha = 0.3
        # self.img = cv2.addWeighted(mask_image_1, alpha, self.img, 1 - alpha, 0)
        # # cv2.drawContours(color_image, draw_contours, -1, (0, 255, 0), 1)
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', self.img)
        # # cv2.imshow('RealSense', red_image)
        # cv2.waitKey(0)

        return node_list

    def run(self):
        if self.contours:
            # Default: bounding contour
            self.contours = sorted(self.contours, key=cv2.contourArea)
            max_cnt = np.array(sorted(self.contours, key=cv2.contourArea)[-1])
            # cv2.drawContours(color_image, max_cnt, -1, (0, 0, 255), 3)

            # only work on the contours/rectangles inside max_cnt
            temp_contours = self.contours[:]
            for i in range(len(temp_contours) - 1, -1, -1):
                cnt = temp_contours[i]
                for j in range(len(cnt)):
                    point = cnt[j]
                    if cv2.pointPolygonTest(max_cnt, (point[0][0], point[0][1]), False) == -1.0:
                        self.contours.pop(i)
                        break

            draw_contours = self.contours[:]  # draw all the contours including max_cnt
            self.contours.pop(-1)  # max_cnt excluded
            self.contours.pop(-1)

            # # Show images
            # cv2.drawContours(self.img, draw_contours, -1, (0, 255, 0), 3)
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', self.img)
            # # cv2.imshow('RealSense', red_image)
            # cv2.waitKey(self.stream)

            nodes = self.list_nodes()

            # motion planning
            motion_planner = Planner(nodes, k=self.pk, r=self.pr)
            # plot cost graph
            G = motion_planner.get_cost_graph()

            graph_img = copy.deepcopy(self.img)
            for n, nbrs in G.adj.items():
                for nbr, eattr in nbrs.items():
                    wt = eattr['weight']
                    node = nodes[n]
                    adj_node = nodes[nbr]
                    graph_img = cv2.line(graph_img, (node[0], node[1]), (adj_node[0], adj_node[1]), color=(0, 255, 0), thickness=1)

            # alpha = 0.3
            # cover_img = cv2.addWeighted(mask_image_1, alpha, color_image, 1 - alpha, 0)

            # # Show images
            # # cv2.drawContours(self.img, draw_contours, -1, (0, 255, 0), 3)
            # cv2.namedWindow('RealSense_Graph', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense_Graph', graph_img)
            # # cv2.imshow('RealSense', red_image)
            # cv2.waitKey(self.stream)

            path, travelled_dst = motion_planner.nearst_neighbor_planning()

            print("path planned")

            # plot path
            mask_image_2 = self.img.copy()
            for k in range(len(path) - 1):
                s = path[k]
                e = path[k + 1]
                mask_image_2 = cv2.line(mask_image_2, (s[0], s[1]), (e[0], e[1]), color=(255, 0, 0),
                                        thickness=2*self.radius)

            alpha = 0.3
            cover_img = cv2.addWeighted(mask_image_2, alpha, self.img, 1 - alpha, 0)

            # Show images
            # cv2.drawContours(color_image, draw_contours, -1, (0, 255, 0), 3)
            cv2.namedWindow('RealSense_Path', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense_Path', cover_img)
            # cv2.imshow('RealSense', red_image)
            cv2.waitKey(self.stream)


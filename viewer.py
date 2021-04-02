## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        # depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # select region of interest
        roi = cv2.selectROI(windowName="roi", img=color_image, showCrosshair=True, fromCenter=False)
        x, y, w, h = roi
        cv2.rectangle(img=color_image, pt1=(x, y), pt2=(x + w, y + h), color=(0, 0, 255), thickness=2)
        color_image = color_image[y:y + h, x:x + w]

        gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(gray_img, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print("length of contours", len(contours))

        # cv2.drawContours(gray_img, contours, -1, (0, 255, 0), 3)
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

            draw_contours = contours
            draw_contours.append(max_cnt)

            cv2.drawContours(color_image, draw_contours, -1, (0, 255, 0), 3)

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            # cv2.imshow('RealSense', red_image)
            cv2.waitKey(1)

        # depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # one_dim = np.ones([color_colormap_dim[0], color_colormap_dim[1], 1])
        # red_image = np.concatenate((168 * one_dim, 50 * one_dim, 50 * one_dim), axis=2)

        # detect the edge of working area
        edges = cv2.Canny(color_image, 60, 180)

        # # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', img)
        # # cv2.imshow('RealSense', red_image)
        # cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
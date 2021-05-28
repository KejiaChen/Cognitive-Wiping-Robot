## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import os
from Cognition import Cognition

transform_matrix =  np.arrat([[-1.02487292,  0.34022334,  0.02018987,  0.53352241],
 [ 0.17523787,  0.87148062, -0.43759237,  0.70475617],
 [-0.0255134 , -0.66172576, -0.5131572 ,  0.49851241],
 [ 0.        ,  0.        ,  0.        ,  1.        ]])


if __name__ == "__main__":
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

    align_to = rs.stream.color
    align = rs.align(align_to)

    # initalize roi
    roi = None

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            depth_intr = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intr = color_frame.profile.as_video_stream_profile().intrinsics
            if not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            l, w, c = color_image.shape

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image,
                                                 dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                 interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))

            def deproject_3d_coordinate(x, y, intrinsic_paramter):
                depth = depth_frame.get_distance(x, y)
                depth_point = rs.rs2_deproject_pixel_to_point(intrinsic_paramter, [x, y], depth)
                return depth_point

            def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
                path = os.path.dirname(os.path.realpath(__file__))
                file = path + "/test_node.txt"
                # file = '/home/kejia/wiping/Cognitive-Wiping-Robot/test_nodes.txt'
                select_list_2d = []
                select_list_3d = []
                if event == cv2.EVENT_LBUTTONDOWN:
                    xy = "%d,%d" % (x, y)
                    print(xy)
                    coord_3d = deproject_3d_coordinate(x, y, depth_intr) # specified in meters
                    rx, ry, rz = coord_3d
                    select_list_2d.append(coord_3d)
                    with open(file, 'a') as f:
                        f.write('%s\n' % coord_3d)
                        f.write('%s\n' % xy)
                    print("point in camera frame:", coord_3d)
                    p_in_cam = np.array(coord_3d.append(1.0))
                    p_in_cam = p_in_cam.reshape((4, 1))
                    p_in_base = transform_matrix.dot(coord_3d)
                    print("point in base frame:", p_in_base)
                    # test_2d = rs.rs2_project_point_to_pixel(color_intr, [rx, ry, rz])
                    cv2.circle(color_image, (x, y), 1, (255, 0, 0), thickness=-1)
                    cv2.putText(color_image, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                                1.0, (0, 0, 0), thickness=1)
                    cv2.imshow("image", color_image)

            # # select region of interest
            # if roi is None:
            #     x_data = []
            #     y_data = []
            #     file = '/home/kejia/wiping/Cognitive-Wiping-Robot/roi_contour.txt'
            #     with open(file) as f:
            #         lines = f.readlines()
            #     for p in lines:
            #         p = p.rstrip("\n")
            #         point_2d = p.split(",")
            #         x_data.append(int(point_2d[0]))
            #         y_data.append(int(point_2d[1]))
            #     mask = np.zeros((l, w), dtype=np.uint8)
            #     x_data = np.array(x_data)
            #     y_data = np.array(y_data)
            #     # x_data = np.array([143, 412, 612, 145])
            #     # y_data = np.array([48, 19, 294, 384])
            #     pts = np.vstack((x_data, y_data)).astype(np.int32).T
            #     cv2.fillPoly(mask, [pts], (255), 8, 0)
            #     # cv2.imshow("mask", mask)
            #
            #     # roi = cv2.selectROI(windowName="roi", img=color_image, showCrosshair=True, fromCenter=False)
            #     # x, y, w, h = roi
            #
            # color_image = cv2.bitwise_and(color_image, color_image, mask=mask)
            # # cv2.rectangle(img=color_image, pt1=(x, y), pt2=(x + w, y + h), color=(0, 0, 255), thickness=2)
            # # color_image = color_image[y:y + h, x:x + w]

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('RealSense', on_EVENT_LBUTTONDOWN)
            cv2.imshow('RealSense', color_image)
            cv2.waitKey(1)


    finally:
        # Stop streaming
        pipeline.stop()

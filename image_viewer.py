## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
from Cognition import Cognition


if __name__ == "__main__":
    real_max = 500

    color_frame = cv2.imread("/home/kejia/Cognitive-Wiping-Robot/static_image_input/ipad_input_words.jpg")

    color_image = np.asanyarray(color_frame)
    color_colormap_dim = color_image.shape
    color_image = cv2.resize(color_image, dsize=(int(color_colormap_dim[1] / 2), int(color_colormap_dim[0] / 2)),
                             interpolation=cv2.INTER_AREA)
    original_image = color_image

    # select region of interest
    roi = cv2.selectROI(windowName="roi", img=color_image, showCrosshair=True, fromCenter=False)
    x, y, w, h = roi
    cv2.rectangle(img=color_image, pt1=(x, y), pt2=(x + w, y + h), color=(0, 0, 255), thickness=2)
    color_image = color_image[y:y + h, x:x + w]

    viewer = Cognition(image=color_image, streaming=0, radius=10)

    viewer.run()
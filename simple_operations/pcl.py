## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##       Align Depth to Color and get data         ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import time
import os
import open3d as o3d

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
# different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
# depth_sensor = profile.get_device().first_depth_sensor()
# depth_scale = depth_sensor.get_depth_scale()
# print("Depth Scale is: ", depth_scale)

# We will be removing the background of objects more than
# clipping_distance_in_meters meters away
# clipping_distance_in_meters = 1  # 1 meter
# clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

save_dir = ""


def check_dir(save_dir):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    return save_dir


n = 0

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x480 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        save_dir = check_dir(save_dir)
        name = str(n)
        depth_image = np.asanyarray(aligned_depth_frame.get_data())

        # print("Saveing img ...")
        # depth_name = name + ".png"
        # image_name = name + ".jpg"
        # cv2.imwrite(os.path.join(save_dir, image_name), color_image)
        # cv2.imwrite(os.path.join(save_dir, depth_name), depth_image)
        # n += 1
        # print("Frame num is %s" % n)
        # if n == 1000:
        #     print("You get 1000 RGBD-imgs!")
        #     break

        cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example', color_image)
        key = cv2.waitKey(1)

        if key == ord("s"):
            print("Saveing img ...")
            depth_name = name + ".png"
            image_name = name + ".jpg"
            n += 1
            cv2.imwrite(os.path.join(save_dir, image_name), color_image)
            cv2.imwrite(os.path.join(save_dir, depth_name), depth_image)
            color_raw = o3d.io.read_image(image_name)
            depth_raw = o3d.io.read_image(depth_name)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
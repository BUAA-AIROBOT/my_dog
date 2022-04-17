#!/usr/bin/python3
import dlib
import cv2
import pyrealsense2 as rs
import numpy as np
import rospy
from std_msgs.msg import Int8


if __name__ == "__main__":
    rospy.init_node("detect")
    pub = rospy.Publisher("detect_control",Int8,queue_size = 1)
    msg = Int8()
    rate = rospy.Rate(500)
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)

    detector = dlib.get_frontal_face_detector()
    color_green = (0, 255, 0)
    line_width = 3
    try:
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            rgb_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            dets = detector(rgb_image)
            op = 0
            for det in dets:
                center = [round((det.left()+det.right())/2), round((det.top()+det.bottom())/2)]
                if center[0] < 240:
                    op += 2
                elif center[0] > 400:
                    op += 1
                else:
                    pass
                depth = depth_image[center[1]][center[0]]
                print(depth)
                if depth > 1000:
                    op += 20
                elif depth < 600:
                    op += 10
                else:
                    pass
                cv2.rectangle(rgb_image, (det.left(), det.top()), (det.right(), det.bottom()), color_green, line_width)
                break
            msg.data = op
            pub.publish(msg)
            cv2.imshow('my webcam', rgb_image)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        pipeline.stop()

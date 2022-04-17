#!/usr/bin/python3
import warnings
warnings.filterwarnings("ignore")
import cv2
import pyrealsense2 as rs
import numpy as np
import rospy
#from yolov5 import YOLOv5
import yolov5
from PIL import Image
from std_msgs.msg import Int8
TMP_IMG_PATH = 'tmp.jpg'
MODEL_DIR = '/home/unitree/.local/lib/python3.6/site-packages/yolov5/weights'
#DEVICE = 'cpu'
KINDS = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
        'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
        'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
        'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
        'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
        'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
        'hair drier', 'toothbrush']
class message_detect():
    def __init__(self, box):
        self.left, self.top, self.right, self.bottom = box
        self.left, self.top, self.right, self.bottom = round(self.left.item()), round(self.top.item()), round(self.right.item()), round(self.bottom.item())

class object_detector():
    def __init__(self, model_name):
        self.model = yolov5.load(MODEL_DIR+'/'+model_name)
        self.object_list = []
        
    def detect(self, img_numpy):
        results = self.model(img_numpy, augment=False)
        predictions = results.pred[0]
        boxes = predictions[:, :4] # x1, y1, x2, y2
        #scores = predictions[:, 4]
        categories = predictions[:, 5]
        self._reset_object_list()
        for box, category in zip(boxes, categories):
            if KINDS[round(category.item())] != 'person':
                continue
            msg = message_detect(box)
            self.object_list.append(msg)

        return self.object_list

    def _reset_object_list(self):
        self.object_list = []

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

    # detector = dlib.get_frontal_face_detector()
    detector = object_detector('yolov5s.pt')
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
            dets = detector.detect(rgb_image)
            op = 0
            for det in dets:
                center = [round((det.left+det.right)/2), round((det.top+det.bottom)/2)]
                if center[0] < 240:
                    op += 2
                elif center[0] > 400:
                    op += 1
                else:
                    pass
                depth = depth_image[center[1]][center[0]]
                if depth > 1200:
                    op += 20
                elif depth < 500:
                    op += 10
                else:
                    pass
                cv2.rectangle(rgb_image, (det.left, det.top), (det.right, det.bottom), color_green, line_width)
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

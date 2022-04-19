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
    def __init__(self, box, depth_image):
        self.left, self.top, self.right, self.bottom = box
        self.left, self.top, self.right, self.bottom = round(self.left.item()), round(self.top.item()), round(self.right.item()), round(self.bottom.item())
        self.center = [round((self.left+self.right)/2), round((self.top+self.bottom)/2)]
        self.depth = np.median(depth_image)#.median()#[self.center[1]][self.center[0]]


class object_detector():
    def __init__(self, model_name):
        self.model = yolov5.load(MODEL_DIR+'/'+model_name)
        self.object = None
        
    def detect(self, rgb_image, depth_image):
        results = self.model(rgb_image, augment=False)
        predictions = results.pred[0]
        person_index = self._get_person_index(predictions)

        boxes = predictions[person_index, :4] # x1, y1, x2, y2
        categories = predictions[person_index, 5]
        print('person detected number:',len(categories))
        
        '''Version 1'''
        # self._reset_object()
        # for box, category in zip(boxes, categories):
        #     if KINDS[round(category.item())] != 'person':
        #         continue
        #     message = message_detect(box, depth_image)
        #     if message.depth == 0:
        #         continue
        #     if self.object==None or message.depth < self.object.depth:
        #         self.object = message
        '''Version 2'''
        print('?')
        if self.object==None:
        #For the first frame
            for box, category in zip(boxes, categories):
                message = message_detect(box, depth_image)
                if message.depth == 0:
                    continue
                if self.object==None or message.depth < self.object.depth:
                    self.object = message
        else:
        #For remaining frames
            print('??')
            alternative_message, error = None, None
            for box, category in zip(boxes, categories):
                message = message_detect(box, depth_image)
                if alternative_message == None: 
                    alternative_message = message
                    error = self._get_error(alternative_message, self.object)
                    print('init_error', error)
                    continue
                tmp_error = self._get_error(message, self.object)
                if tmp_error < error:
                    error = tmp_error
                    alternative_message = message
                print('tmp_error',tmp_error)
            self.object = alternative_message
        return self.object

    def _reset_object(self):
        self.object = None

    def _get_person_index(self, predictions):
        return predictions[:, 5] == 0.

    def _get_error(self, message1, message2):
        # print(message1.center, message1.depth)
        return abs(sum(message1.center)-sum(message2.center))#+abs(message1.depth-message2.depth)

if __name__ == "__main__":
    rospy.init_node("detect")
    pub = rospy.Publisher("detect_control",Int8,queue_size = 1)
    msg = Int8()
    #rate = rospy.Rate(500)
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)

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
            det = detector.detect(rgb_image, depth_image)
            op = 0
            if det != None:
                if det.center[0] < 240:
                    op += 2
                elif det.center[0] > 400:
                    op += 1
                else:
                    pass
                if det.depth > 1200:
                    op += 20
                elif det.depth < 500:
                    op += 10
                else:
                    pass
                # print(det.depth)
                cv2.rectangle(rgb_image, (det.left, det.top), (det.right, det.bottom), color_green, line_width)
            msg.data = op
            pub.publish(msg)
            cv2.imshow('my webcam', rgb_image)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                while True:
                    msg.data = 0
                    pub.publish(msg)
    finally:
        # Stop streaming
        pipeline.stop()

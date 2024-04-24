#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse

# Camera capture
from cv_bridge import CvBridge,CvBridgeError

import cv2
import numpy as np 

# import mutex
from threading import Lock

from roboflow import Roboflow

class YOLOVisualizer:
    def __init__(self, cam_spec : str = "mounted_cam"):
        self.bridge = CvBridge()
        self.rf = Roboflow(api_key="ityuNxTBT027SO7GMT0Y")
        self.project = self.rf.workspace().project("zeolitepellets")
        self.model = self.project.version(1).model

        self.rgb_img_sub = rospy.Subscriber(f"/camera/color/image_raw", Image, self.rgb_img_callback, queue_size=1)
        self.run_yolo_srv = rospy.Service("run_yolo", Trigger, self.run_yolo_srv)
        self.clear_viz_srv = rospy.Service("clear_yolo", Trigger, self.clear_viz_srv)
        self.image_pub = rospy.Publisher('yolo_image', Image, queue_size=1)

        self.predictions = None
        self.current_img = None
        self.predictions_lock = Lock()
        self.current_img_lock = Lock()

    def rgb_img_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # set current_img
        self.current_img_lock.acquire()
        self.current_img = frame
        self.current_img_lock.release()

        # retreive predictions
        self.predictions_lock.acquire()
        predictions = self.predictions
        self.predictions_lock.release()
        
        if predictions is not None and predictions['predictions']:
            # Add bounding boxes to the model prediction of connector
            for bounding_box in predictions["predictions"]:
                x0 = bounding_box['x'] - bounding_box['width']  / 2
                x1 = bounding_box['x'] + bounding_box['width']  / 2
                y0 = bounding_box['y'] - bounding_box['height'] / 2
                y1 = bounding_box['y'] + bounding_box['height'] / 2
                
                start_point = (int(x0), int(y0))
                endpoint = (int(x1), int(y1))
                cv2.rectangle(frame, start_point, endpoint, color=(0,255,0), thickness=2)
        
        # Display the resulting frame
        ros_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.image_pub.publish(ros_img)

    def run_yolo_srv(self, req):
        self.current_img_lock.acquire()
        frame = self.current_img
        self.current_img_lock.release()
        # infer on a local image
        predictions = self.model.predict(frame, confidence=40, overlap=30).json()

        self.predictions_lock.acquire()
        self.predictions = predictions
        self.predictions_lock.release()

        return TriggerResponse(success=True, message="YOLO ran successfully")
    
    def clear_viz_srv(self, req):
        self.predictions_lock.acquire()
        self.predictions = None
        self.predictions_lock.release()

        return TriggerResponse(success=True, message="YOLO visualization cleared")


def main():
    rospy.init_node("ml_tracker",anonymous=True)
    yolo_viz = YOLOVisualizer()
    rospy.spin()

if __name__ == '__main__':
    main()
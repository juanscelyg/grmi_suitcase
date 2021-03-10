#!/usr/bin/env python

import numpy as np
import rospy
import logging
import time
import cv2
import os
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D

class GRMI_detector():
    def __init__(self):
        # ROS TIMER
        self.mytime = 0.75

        # YOLO Config
        self.yolo_path = rospy.get_param('/person_detector/yolo_path')
        self.weightsPath = os.path.sep.join([self.yolo_path, "yolov3-tiny.weights"])
        self.configPath = os.path.sep.join([self.yolo_path, "yolov3-tiny.cfg"])
        self.labelsPath = os.path.sep.join([self.yolo_path, "coco.names"])
        self.confidence = 0.5
        self.threshold = 0.3
        self.LABELS = open(self.labelsPath).read().strip().split("\n")
        self.color = [0,0,205] #Rojitoo
        rospy.loginfo("loading YOLO from disk...")
        self.net = cv2.dnn.readNetFromDarknet(self.configPath, self.weightsPath)

        # ROS INFRAESTRUCRE
        self.cv_image = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher("/detected_image",Image, queue_size=1)
        self.error_pub = rospy.Publisher("/error_pose",Pose2D, queue_size=1)

    def callback(self, msg_image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        except CvBridgeError as e:
            print(e)

    def detect(self, event):
        _cv_image = self.cv_image
        (H, W) = _cv_image.shape[:2]
        ln = self.net.getLayerNames()
        ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        blob = cv2.dnn.blobFromImage(self.cv_image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        start = time.time()
        layerOutputs = self.net.forward(ln)
        end = time.time()
        #rospy.loginfo("YOLO took {:.6f} seconds at {:.3f}".format(end - start, end))
        boxes = []
        confidences = []
        classIDs = []
        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                if confidence > self.confidence:
                	box = detection[0:4] * np.array([W, H, W, H])
                	(centerX, centerY, width, height) = box.astype("int")
                	x = int(centerX - (width / 2))
                	y = int(centerY - (height / 2))
                	boxes.append([x, y, int(width), int(height)])
                	confidences.append(float(confidence))
                	classIDs.append(classID)
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence,self.threshold)
        if len(idxs) > 0:
            i=0 #Solo la primera deteccion
            if classIDs[i]==0:
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                #rospy.loginfo(self.color)
                cv2.rectangle(_cv_image, (x, y), (x + w, y + h), self.color, 2)
                #cv2.circle(_cv_image, (W/2, H/2), 3, (0, 255, 0), -1)
                error_angle = (W/2.0 - (x + w/2.0))/W
                error_dist = (H*0.75 - (h))/H
                text = "{} {}: {:.2f}{}".format(self.LABELS[classIDs[i]],i, confidences[i]*100, '%')
                cv2.putText(_cv_image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, self.color, 2)
                rospy.loginfo("Detected: "+text)
        else:
            error_angle = 0.0
            error_dist = 0.0
        try:
            self.pub_error(error_dist, error_angle)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(_cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


    def pub_error(self, error_distance, error_angle):
        pose_msg = Pose2D()
        pose_msg.x = error_distance
        pose_msg.theta = error_angle
        self.error_pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('person_detector')
    try:
        node = GRMI_detector()
        rospy.Timer(rospy.Duration(node.mytime), node.detect)
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

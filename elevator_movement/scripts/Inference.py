#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, String
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from ultralytics import YOLO


class Node(object):
    def __init__(self):
        # Params
        self.out = "Door closed"
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', String, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image_color", Image, self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        model = YOLO('best.pt')
        results = model(self.br.imgmsg_to_cv2(msg))

        for result in results:
            for box in result.boxes:
                if box.data[0][5] == 2:
                    if box.data[0][3] - box.data[0][1] > 1000 and box.data[0][2] - box.data[0][0] > 600:
                        self.out = "Door open"
                    else:
                        self.out = "Door closed"

    def start(self):
        rospy.loginfo("Timing images")
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            # br = CvBridge()
            if self.image is not None:
                self.pub.publish(self.out)
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Node()
    my_node.start()

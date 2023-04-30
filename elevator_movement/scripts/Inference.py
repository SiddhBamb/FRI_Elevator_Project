#!/usr/bin/env python

# # import rospy
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import cv2
# import os
# import numpy as np


# class Node(object):
#     def __init__(self):
#         rospy.loginfo("Node(object)")
#         print("Node(object)")
#         # Params
#         self.out = "Door closed"
#         self.br = CvBridge()
#         # Node cycle rate (in Hz).
#         self.loop_rate = rospy.Rate(10)

#         # Publishers
#         self.pub = rospy.Publisher('elevator_door_classification', String, queue_size=10)

#         # Subscribers
#         rospy.Subscriber("/rgb/image_raw", Image, self.callback)

#     def callback(self, msg):
#         rospy.loginfo('Image received...')
#         model = YOLO('best.pt')
#         results = model(self.br.imgmsg_to_cv2(msg))

#         for result in results:
#             for box in result.boxes:
#                 if box.data[0][5] == 2:
#                     if box.data[0][3] - box.data[0][1] > 1000 and box.data[0][2] - box.data[0][0] > 600:
#                         self.out = "Door open"
#                     else:
#                         self.out = "Door closed"

#     def start(self):
#         rospy.loginfo("Timing images")
#         print("OMG")
#         # rospy.spin()
#         while not rospy.is_shutdown():
#             rospy.loginfo('publishing image')
#             # br = CvBridge()
#             if self.out is not None:
#                 self.pub.publish(self.out)
#             self.loop_rate.sleep()
#             rospy.spinOnce()


# if __name__ == '__main__':
#     print("OMG LOL")
    # rospy.init_node('Inference', anonymous=True)
    # rospy.loginfo("__main__")
    # my_node = Node()
    # my_node.start()

from ultralytics import YOLO

import pykinect_azure as pykinect

import roslibpy

import cv2

import time

class Classify(object):
    def __init__(self):
        self.model = YOLO('best.pt')

    def doClassify(self, in_image):
        results = self.model(in_image)
        for result in results:
            for box in result.boxes:
                if box.data[0][5] == 2:
                    #if box.data[0][3] - box.data[0][1] > 300 and box.data[0][2] - box.data[0][0] > 150:
                    return "Door open"
        return "Door closed"


if __name__ == "__main__":
    classify = Classify()

    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()
    talker = roslibpy.Topic(client, '/door_openness', 'std_msgs/String')

    # Initialize the library, if the library is not found, add the library path as argument
    pykinect.initialize_libraries()

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    # print(device_config)

    # Start device
    device = pykinect.start_device(config=device_config)

    cv2.namedWindow('Color Image',cv2.WINDOW_NORMAL)
    while True:

        # Get capture
        capture = device.update()

        # Get the color image from the capture
        ret, color_image = capture.get_color_image()

        if not ret:
            continue

        # Plot the image
        cv2.imshow("Color Image",color_image)

        is_the_door_open = classify.doClassify(color_image)
        print(is_the_door_open)
        talker.publish(roslibpy.Message({'data': is_the_door_open}))

        # Press q key to stop
        if cv2.waitKey(1) == ord('q'): 
            break
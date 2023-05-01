# !/usr/bin/env python

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

    def doExtClassify(self, in_image):
        results = self.model(in_image)
        for result in results:
            for box in result.boxes:
                if box.data[0][5] == 2:
                    # if box.data[0][3] - box.data[0][1] > 300 and box.data[0][2] - box.data[0][0] > 150:
                    return "Door open"
        return "Door closed"

    def doIntClassify(self, in_image):
        b, g, r = cv2.split(in_image)
        sumSquaredDif = 0
        for i in range(b.shape[0]):
            for j in range(b.shape[1]):
                sumSquaredDif += pow(128 - (b[i, j]), 2)
        for i in range(g.shape[0]):
            for j in range(g.shape[1]):
                sumSquaredDif += pow(128 - (g[i, j]), 2)
        for i in range(r.shape[0]):
            for j in range(r.shape[1]):
                sumSquaredDif += pow(128 - (r[i, j]), 2)
        avg = sumSquaredDif / (3 * b.rows * b.cols)

        if avg > 5000:
            return "Door open"
        else:
            return "Door closed"

    # constants

    # //Expected messages for door open/closed
    # const std::string door_open_msg = "Door open";
    # const std::string door_closed_msg = "Door closed";

    # //Threshold for determining if door is open
    # const double THRESHOLD = 5000; //already tuned

    # door detector from inside method

    # std::vector<cv::Mat> channels;
    # cv::split(img, channels);    //img is original image

    # double sumSquaredDiff = 0;
    # int totalCount = 0;
    # for (cv::Mat currChannel : channels) {
    #     for (int i = 0; i < currChannel.rows; i++) {
    #         for (int j = 0; j < currChannel.cols; j++) {
    #             sumSquaredDiff += pow(128 - (int) (currChannel.at<uchar>(i,j)), 2);
    #             totalCount++;
    #         }
    #     }
    # }
    # double avgSquaredDiff = sumSquaredDiff / totalCount;

    # std_msgs::String msgToPublish;
    # if (avgSquaredDiff > THRESHOLD) {
    #     msgToPublish.data = door_open_msg;
    # } else {
    #     msgToPublish.data = door_closed_msg;
    # }
    # ROS_INFO("door detection from inside: %s, value = %f", msgToPublish.data.c_str(), avgSquaredDiff);
    # publisher.publish(msgToPublish);

    # probably can return string instead of publishing here
    # publish on this channel: elevator_door_open_from_inside


if __name__ == "__main__":
    classify = Classify()

    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()
    exterior = roslibpy.Topic(client, '/ext_door_openness', 'std_msgs/String')
    interior = roslibpy.Topic(client, '/int_door_openness', 'std_msgs/String')

    # Initialize the library, if the library is not found, add the library path as argument
    pykinect.initialize_libraries()

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    # print(device_config)

    # Start device
    device = pykinect.start_device(config=device_config)

    cv2.namedWindow('Color Image', cv2.WINDOW_NORMAL)
    while True:

        # Get capture
        capture = device.update()

        # Get the color image from the capture
        ret, color_image = capture.get_color_image()

        if not ret:
            continue

        # Plot the image
        cv2.imshow("Color Image", color_image)

        ext_is_the_door_open = classify.doExtClassify(color_image)
        int_is_the_door_open = classify.doIntClassify(color_image)
        # print(ext_is_the_door_open, int_is_the_door_open)
        exterior.publish(roslibpy.Message({'data': ext_is_the_door_open}))
        interior.publish(roslibpy.Message({'data': int_is_the_door_open}))

        # Press q key to stop
        if cv2.waitKey(1) == ord('q'):
            break

#include <string>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>


//Expected messages for door open/closed
const std::string door_open_msg = "Door open";
const std::string door_closed_msg = "Door closed";

//Threshold for determining if door is open
const double THRESHOLD = 10000;

//Publisher object
ros::Publisher publisher;


//Callback function when a new image from camera is received
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr opencv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = opencv_img_ptr->image;
    
    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    double sumSquaredDiff = 0;
    int totalCount = 0;
    for (cv::Mat currChannel : channels) {
        for (int i = 0; i < currChannel.rows; i++) {
            for (int j = 0; j < currChannel.cols; j++) {
                sumSquaredDiff = pow(currChannel.at<uchar>(i,j) - 128, 2);
                totalCount++;
            }
        }
    }
    double avgSquaredDiff = sumSquaredDiff / totalCount;

    std_msgs::String msgToPublish;
    if (avgSquaredDiff > THRESHOLD) {
        msgToPublish.data = door_open_msg;
    } else {
        msgToPublish.data = door_closed_msg;
    }
    ROS_INFO("door detection from inside: %s", msgToPublish.data.c_str());
    publisher.publish(msgToPublish);

}


int main(int argc, char **argv)
{

    //Initialize ros
    ros::init(argc, argv, "door_detector");
    ros::NodeHandle n;

    //Subscribe to the two topics
    ros::Subscriber classificationSubscriber = n.subscribe("/rgb/image_raw", 1, imageCallback);

    //Publish on channel
    publisher = n.advertise<std_msgs::String>("elevator_door_open_from_inside", 1);

    //Spin
    ros::spin();

    return 0;

}
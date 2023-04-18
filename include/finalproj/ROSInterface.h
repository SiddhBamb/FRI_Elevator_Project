#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include "ColorFilter.h"

class ROSInterface {
protected:
    ros::NodeHandle _handle;
    image_transport::ImageTransport _imgTransport;
    image_transport::Publisher _blue, _green, _red, _bgr;
    image_transport::Subscriber _cupVideo;

    ColorFilter _colorFilter;

public:
    ROSInterface(ColorFilter colorFilter);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
};

#endif
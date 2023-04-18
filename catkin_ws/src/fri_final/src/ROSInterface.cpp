#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "../include/fri_final/ROSInterface.h"


ROSInterface::ROSInterface(ColorFilter colorFilter) : _colorFilter(colorFilter), _handle(), _imgTransport(_handle) {

    _cupVideo = _imgTransport.subscribe("/kinect2/hd/image_color", 1, &ROSInterface::imageCallback, this);
    _blue = _imgTransport.advertise("blue", 1);
    _green = _imgTransport.advertise("green", 1);
    _red = _imgTransport.advertise("red", 1);
    _bgr = _imgTransport.advertise("bgr", 1);

}

void ROSInterface::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr opencv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    _colorFilter.processImage(opencv_img_ptr->image);
    
    sensor_msgs::ImagePtr blueMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _colorFilter.getBlueImage()).toImageMsg();
    _blue.publish(blueMsg);

    sensor_msgs::ImagePtr greenMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _colorFilter.getGreenImage()).toImageMsg();
    _green.publish(greenMsg);

    sensor_msgs::ImagePtr redMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _colorFilter.getRedImage()).toImageMsg();
    _red.publish(redMsg);

    sensor_msgs::ImagePtr bgrMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _colorFilter.getBGRImage()).toImageMsg();
    _bgr.publish(bgrMsg);

}
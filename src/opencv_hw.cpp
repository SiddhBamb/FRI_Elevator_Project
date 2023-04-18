#include "../include/opencv_hw/ColorFilter.h"
#include "../include/opencv_hw/ROSInterface.h"

using namespace std;
using namespace cv;

int main(int arvc, char **argv) {

    ros::init(arvc, argv, "colored_cups");
    ColorFilter cf;
    ROSInterface rosInterface(cf);
    ros::spin();

}
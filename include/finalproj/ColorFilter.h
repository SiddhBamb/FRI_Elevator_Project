#ifndef COLOR_FILTER_H
#define COLOR_FILTER_H

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

class ColorFilter {
protected:
    cv::Mat _frame;
    cv::Mat _blueSub, _greenSub, _redSub;
    cv::Mat _blueThreshold, _greenThreshold, _redThreshold;
    cv::Mat _blueMask, _greenMask, _redMask;
    cv::Mat _blueCup, _greenCup, _redCup;
    cv::Mat _allCups;

    std::vector<cv::Mat> _chans;

public:
    void processImage(cv::Mat img);

    void split();

    void findBlue();
    void findGreen();
    void findRed();
    void findBGR();

    void showResult();

    cv::Mat getBlueImage();
    cv::Mat getGreenImage();
    cv::Mat getRedImage();
    cv::Mat getBGRImage();
};

#endif
#include "../include/fri_final/ColorFilter.h"

using namespace std;
using namespace cv;

void ColorFilter::processImage(cv::Mat img) {
    _frame = img;
    split();
    findBlue();
    findGreen();
    findRed();
    findBGR();
    //showResult();
}

void ColorFilter::split() {
    cv::split(_frame, _chans);
}

void ColorFilter::showResult() {

    //PROBLEM 1
    //cv::imshow("Video output", _frame);


    //PROBLEM 2: B
    //cv::imshow("Video output", _chans[0]);

    //PROBLEM 2: G
    //cv::imshow("Video output", _chans[1]);

    //PROBLEM 2: R
    //cv::imshow("Video output", _chans[2]);


    //PROBLEM 3: Blue Subtraction
    //cv::imshow("Video output", _blueSub);

    //PROBLEM 3: Blue Threshold
    //cv::imshow("Video output", _blueThreshold);

    //PROBLEM 3: Largest Blue Blob
    //cv::imshow("Video output", _blueMask);


    //PROBLEM 4: Green Subtraction
    //cv::imshow("Video output", _greenSub);

    //PROBLEM 4: Green Threshold
    //cv::imshow("Video output", _greenThreshold);

    //PROBLEM 4: Largest Green Blob
    //cv::imshow("Video output", _greenMask);


    //PROBLEM 5: Red Subtraction
    //cv::imshow("Video output", _redSub);

    //PROBLEM 5: Red Threshold
    //cv::imshow("Video output", _redThreshold);

    //PROBLEM 5: Largest Red Blob
    //cv::imshow("Video output", _redMask);


    //PROBLEM 6: ALL THREE
    //cv::imshow("Video output", _allCups);


    //cv::waitKey(50);

}

void ColorFilter::findBlue() {

    cv::subtract(_chans[0], _chans[2], _blueSub);

    cv::threshold(_blueSub, _blueThreshold, 55, 255, cv::THRESH_BINARY);

    std::vector<cv::Mat> blobs;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(_blueThreshold, blobs, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    int largestIndex = 0;
    for (int i = 1; i < blobs.size(); i++) {
        if (cv::contourArea(blobs[i]) > cv::contourArea(blobs[largestIndex])) {
            largestIndex = i;
        }
    }
    _blueMask = cv::Mat::zeros(_frame.rows, _frame.cols, CV_8UC1);
    cv::drawContours(_blueMask, blobs, largestIndex, cv::Scalar(255), cv::LineTypes::FILLED, 8, hierarchy);

    _blueCup = cv::Mat::zeros(_frame.rows, _frame.cols, CV_8UC3);
    _frame.copyTo(_blueCup, _blueMask);

}

void ColorFilter::findGreen() {

    cv::subtract(_chans[1], _chans[0], _greenSub);

    cv::threshold(_greenSub, _greenThreshold, 80, 255, cv::THRESH_BINARY);

    std::vector<cv::Mat> blobs;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(_greenThreshold, blobs, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    int largestIndex = 0;
    for (int i = 1; i < blobs.size(); i++) {
        if (cv::contourArea(blobs[i]) > cv::contourArea(blobs[largestIndex])) {
            largestIndex = i;
        }
    }
    _greenMask = cv::Mat::zeros(_frame.rows, _frame.cols, CV_8UC1);
    cv::drawContours(_greenMask, blobs, largestIndex, cv::Scalar(255), cv::LineTypes::FILLED, 8, hierarchy);

    _greenCup = cv::Mat::zeros(_frame.rows, _frame.cols, CV_8UC3);
    _frame.copyTo(_greenCup, _greenMask);

}

void ColorFilter::findRed() {

    cv::subtract(_chans[2], _chans[1], _redSub);

    cv::threshold(_redSub, _redThreshold, 60, 255, cv::THRESH_BINARY);

    std::vector<cv::Mat> blobs;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(_redThreshold, blobs, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    int largestIndex = 0;
    for (int i = 1; i < blobs.size(); i++) {
        if (cv::contourArea(blobs[i]) > cv::contourArea(blobs[largestIndex])) {
            largestIndex = i;
        }
    }
    _redMask = cv::Mat::zeros(_frame.rows, _frame.cols, CV_8UC1);
    cv::drawContours(_redMask, blobs, largestIndex, cv::Scalar(255), cv::LineTypes::FILLED, 8, hierarchy);

    _redCup = cv::Mat::zeros(_frame.rows, _frame.cols, CV_8UC3);
    _frame.copyTo(_redCup, _redMask);
    
}

void ColorFilter::findBGR() {

    _allCups = cv::Mat::zeros(_frame.rows, _frame.cols, CV_8UC3);
    cv::bitwise_or(_blueCup, _allCups, _allCups);
    cv::bitwise_or(_greenCup, _allCups, _allCups);
    cv::bitwise_or(_redCup, _allCups, _allCups);

}

cv::Mat ColorFilter::getBlueImage() {
    return _blueCup;
}

cv::Mat ColorFilter::getGreenImage() {
    return _greenCup;
}

cv::Mat ColorFilter::getRedImage() {
    return _redCup;
}

cv::Mat ColorFilter::getBGRImage() {
    return _allCups;
}
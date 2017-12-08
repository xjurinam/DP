/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   HSPointDetector.h
 * Author: marek
 *
 * Created on November 30, 2017, 1:55 PM
 */

#ifndef HSPOINTDETECTOR_H
#define HSPOINTDETECTOR_H

#include <opencv2/opencv.hpp>


class HSPointDetector {
public:
    HSPointDetector();
    HSPointDetector(const std::string &path);
    HSPointDetector(const HSPointDetector& orig);
    virtual ~HSPointDetector();
private:
    bool is_detected;
    bool is_empty;
    cv::Mat object;
    cv::Ptr<cv::ml::KNearest> knn;
};

#endif /* HSPOINTDETECTOR_H */


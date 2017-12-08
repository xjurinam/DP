/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   HSPointDetector.cpp
 * Author: marek
 * 
 * Created on November 30, 2017, 1:55 PM
 */

#include "HSPointDetector.h"

HSPointDetector::HSPointDetector() {
    this->is_detected = false;
    this->is_empty = false;
}

HSPointDetector::HSPointDetector(const std::string &path) {
    cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::loadFromCSV(
            path,
            1,-1,-1,cv::String(),';');
    this->knn = cv::ml::KNearest::create();
    this->knn->train(td);
    this->is_detected = false;
    this->is_empty = false;
}

HSPointDetector::HSPointDetector(const HSPointDetector& orig) {
}

HSPointDetector::~HSPointDetector() {
}

/*
    Ptr<cv::ml::TrainData> td = cv::ml::TrainData::loadFromCSV("example.csv",1,-1,-1,String(),';');
    knn->train(td);
 */
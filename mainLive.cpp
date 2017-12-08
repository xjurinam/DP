/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   mainLive.cpp
 * Author: marek
 *
 * Created on November 16, 2017, 12:09 PM
 */

/*
    Rozmery HS:
 * 
 * 1    2
 * 
 * 3    4   5
 *          6
 *          7
 * 1 - 2 = 10,7 cm
 * 1 - 3 = 11 cm
 * 2 - 4 = 11 cm
 * 3 - 4 = 10,7 cm
 * 
 */

#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <semaphore.h>

#include <iostream>
#include <fstream>

#include <dirent.h>

using namespace std;
using namespace cv;

int low_h=60, low_s=150, low_v=65;
int high_h=179, high_s=255, high_v=255;

typedef struct sem{
    sem_t mutex;
} SEM;

class HSDetect {
public:
    HSDetect(){
        this->is_detected = false;
        this->is_empty = true;
    };
    
    HSDetect(const string &path){
        this->is_detected = false;
        this->classifier.load(path);
        if(this->classifier.empty())
            this->is_empty = true;
        else
            this->is_empty = false;
    };
    
    bool empty(){
        if(this->is_empty)
            return true;
        else
            return false;
    }
    
    bool detected(){
        if(this->is_detected)
            return true;
        else
            return false;
    }
    
    void detect(cv::Mat &input){
        std::vector<cv::Rect> objects;
        cv::Rect rectangle(Point(0,0),Size(0,0));
        
        this->classifier.detectMultiScale(input, objects);
        if(objects.size() == 0){
            this->is_detected = false;
        }
        else{
            for (int i = 0; i < (int) objects.size(); i++) {
                if(rectangle.area() < objects[i].area()){
                    rectangle = objects[i];
                }
            }
            this->is_detected = true;
            object = input.colRange(rectangle.x,rectangle.x+rectangle.width);
            object = input.rowRange(rectangle.y,rectangle.y+rectangle.height);
            offset = rectangle;
        }
    }
    
    cv::Mat & getObject(){
        return this->object;
    }
    
private:
    cv::Mat object;
    cv::CascadeClassifier classifier;
    cv::Rect offset;
    bool is_detected;
    bool is_empty;
    
    HSDetect(const HSDetect& orig);
    virtual ~HSDetect();
};

int initialize_semaphores(SEM *sem){
    int result = sem_init(&(sem->mutex), 0, 1);
    if(result != 0)
        return result;
    
    return 0;
}

int object_detection(SEM &sem){
    
}

double euclid_distance(Point a, Point b){
    double result = std::sqrt((pow((a.x-b.x),2)+pow((a.y-b.y),2)));
}

void on_low_h_thresh_trackbar(int, void *)
{
    low_h = min(high_h-1, low_h);
    setTrackbarPos("Low H","HSV Treshold", low_h);
}
void on_high_h_thresh_trackbar(int, void *)
{
    high_h = max(high_h, low_h+1);
    setTrackbarPos("High H", "HSV Treshold", high_h);
}
void on_low_s_thresh_trackbar(int, void *)
{
    low_s = min(high_s-1, low_s);
    setTrackbarPos("Low S","HSV Treshold", low_s);
}
void on_high_s_thresh_trackbar(int, void *)
{
    high_s = max(high_s, low_s+1);
    setTrackbarPos("High S", "HSV Treshold", high_s);
}
void on_low_v_thresh_trackbar(int, void *)
{
    low_v= min(high_v-1, low_v);
    setTrackbarPos("Low V","HSV Treshold", low_v);
}
void on_high_v_thresh_trackbar(int, void *)
{
    high_v = max(high_v, low_v+1);
    setTrackbarPos("High V", "HSV Treshold", high_v);
}

/*
 * 
 */
int main(int argc, char** argv) {
    
//    cv::VideoCapture cap(0);
    cv::VideoCapture cap("Video/vlc-record-01.avi");
//    cv::VideoCapture cap("Video/WP_20171013_010.mp4");

    if(!cap.isOpened()){
        std::cout << "Video file doesnt load!" << std::endl;
        return EXIT_FAILURE;
    }
    
    /* OBJECT DETECTION */
    cv::CascadeClassifier objectDetector;
    objectDetector.load("cascade/cascade.xml");
    if(objectDetector.empty()){
        std::cout << "Empty model!" << std::endl;
        return EXIT_FAILURE;
    }
    
    /* POINT DETECTION */
    Ptr<cv::ml::KNearest> knn = cv::ml::KNearest::create();
    Ptr<cv::ml::TrainData> td = cv::ml::TrainData::loadFromCSV("example.csv",1,-1,-1,String(),';');
    knn->train(td);
    
    int c_fps = cap.get(cv::CAP_PROP_FPS);
    int c_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int c_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    
    std::cout << "FPS:\t" << c_fps << std::endl 
            << "Video width:\t" << c_width << std::endl
            << "Video height:\t" << c_height << std::endl;
    
    double winX = 640.0*1.2;
    double winY = 480.0*1.2;
    double scaleX = winX / c_width;
    double scaleY = winY / c_height;
    
    cv::Mat f_src, f_resized, f_roi, f_blur, f_gray, f_canny, f_sectors;
    cv::Rect o_best(Point(0,0),Size(0,0));
    vector<cv::Point> m_points(7);
    int f_id = 0;
    bool is_detected = false;
    
    /*SEM *sem = new SEM;
    if(initialize_semaphores(sem) != 0){
        std::cout << "Semaphores initialization failed!" << std::endl;
        return EXIT_FAILURE;
    }*/
    
    cv::namedWindow("src");     cv::moveWindow("src", 0, 0);
    cv::namedWindow("HSV Treshold");     cv::moveWindow("HSV Treshold", winX, 0);
    
    createTrackbar("Low H","HSV Treshold", &low_h, 179, on_low_h_thresh_trackbar);
    createTrackbar("High H","HSV Treshold", &high_h, 179, on_high_h_thresh_trackbar);
    
    createTrackbar("Low S","HSV Treshold", &low_s, 255, on_low_s_thresh_trackbar);
    createTrackbar("High S","HSV Treshold", &high_s, 255, on_high_s_thresh_trackbar);
    
    createTrackbar("Low V","HSV Treshold", &low_v, 255, on_low_v_thresh_trackbar);
    createTrackbar("High V","HSV Treshold", &high_v, 255, on_high_v_thresh_trackbar);
    
    cap.set(cv::CAP_PROP_POS_FRAMES,(int)80);
        
    while(cap.read(f_src)){
        cv::TickMeter tm;
        tm.start();
        
        cv::resize(f_src, f_resized, cv::Size(), scaleX, scaleY);
//        cv::rotate(f_resized, f_resized, ROTATE_90_CLOCKWISE);
//        cv::GaussianBlur(f_resized, f_resized, Size(9,9), 0, 0);
  
        /* OBJECT DETECTION */
        if(!is_detected || f_id % 10 == 0){
            cv::Mat circlesMat(Size(0,0),CV_32FC1);
            
            std::vector<cv::Rect> objects;
            objectDetector.detectMultiScale(f_resized, objects, 1.1,5);
            for (int i = 0; i < (int) objects.size(); i++) {
                if(o_best.area() < objects[i].area())
                    o_best = objects[i];
            }
            if(objects.size() == 0)
                continue;
            f_roi = f_resized.colRange(o_best.x, o_best.x+o_best.width);
            f_roi = f_roi.rowRange(o_best.y, o_best.y+o_best.width);
            
            cv::GaussianBlur(f_roi, f_blur, Size(9,9), 0, 0);
    //        cv::cvtColor(f_roi, f_gray, cv::COLOR_BGR2GRAY);
    //        cv::equalizeHist(f_gray,f_gray);
            cv::Canny(f_blur, f_canny, 50, 100, 3, true);
            
            vector<vector<cv::Point>> contours;
            cv::findContours(f_canny, contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
            Mat drawing = Mat::zeros( f_canny.size(), CV_8UC3 );
            vector<RotatedRect> minRect( contours.size() );
            int count = 0;

            for( int i = 0; i< contours.size(); i++ ){
                minRect[i] = minAreaRect( Mat(contours[i]) );
                vector<cv::Point> body;
                cv::approxPolyDP(contours[i], body, 0.001, true);
                int sucet = minRect[i].size.height + minRect[i].size.width;

                if(body.size() > 5 && minRect[i].size.width < 2*minRect[i].size.height
                        && minRect[i].size.height < 2*minRect[i].size.width
                        && body.size() > (0.80*sucet) /*&& sucet < 12*/){


    //                drawContours( /*drawing*/f_resized, contours, i, Scalar( 0, 0, 255), 1, 8, vector<Vec4i>(), 0, Point(o_best.x,o_best.y));
                    cv::circle(drawing,minRect[i].center,1, Scalar(0,255,255));
                    cv::Mat hh(1,2,CV_32FC1);
                    hh.at<float>(0,0) = minRect[i].center.x;
                    hh.at<float>(0,1) = minRect[i].center.y;
                    circlesMat.push_back(hh);

                    Mat predictedMat;
                    knn->findNearest(hh,8,predictedMat);
                    int predictedClass =(int) predictedMat.at<float>(0,0);
                    if(predictedClass > 0 && predictedClass < 7)
                        m_points[predictedClass-1]=minRect[i].center;
                    count++;
                }
            }
            /*Mat predictedMat;
            knn->findNearest(circlesMat,8,predictedMat);
            while(true){
                bool is_end = true;
                int index = -1;
                int counter = 0;
                int number = -1;
                std::vector<int> indexes;

                for(int h = 0; h < predictedMat.rows; h++){
                    int predictedNumber = (int) predictedMat.at<float>(h);
                    if(predictedNumber != 0){
                        if(predictedNumber > 0 && predictedNumber < 8){
                            if(number == -1){
                                number = predictedNumber;
                                counter++;
                                index = h;
                                indexes.push_back(h);
                            }
                            else if(predictedNumber == number){
                                counter++;
                                index = h;
                                indexes.push_back(h);
                            }
                        }
                        else
                            predictedMat.at<float>(h) = 0;
                    }
                }

                if(indexes.size() == 1){
                    m_points[number-1] = Point((int)circlesMat.at<float>(index,0),(int)circlesMat.at<float>(index,1));
                    predictedMat.at<float>(index) = 0;
                }

                for(int h = 0; h < predictedMat.rows; h++)
                    if(predictedMat.at<float>(h) != 0){
                        is_end = false;
                        break;
                    }
                if(is_end)
                    break;
            }*/
            
            if(m_points[0].x != 0 && m_points[0].y != 0
                    && m_points[1].x != 0 && m_points[1].y != 0
                    && m_points[2].x != 0 && m_points[2].y != 0
                    && m_points[3].x != 0 && m_points[3].y != 0)
                is_detected = true;
        }
        
        if(is_detected){
            double realSize = 110.0; // Skutocna vzdialenost bodov 1 a 3 v milimetroch
            Mat f_hsv, f_blur2, f_help;
            cv::GaussianBlur(f_roi, f_blur2, Size(9,9), 0, 0);
            cv::cvtColor(f_blur2, f_hsv, cv::COLOR_BGR2HSV);
            inRange(f_hsv, Scalar(low_h,low_s,low_v), Scalar(high_h,high_s,high_v), f_sectors);
            cv::erode(f_sectors, f_sectors, Mat());
            cv::dilate(f_sectors, f_sectors, Mat()); 
            
            f_help = f_sectors.clone();
            vector<vector<cv::Point>> contours2;
            cv::findContours(f_help, contours2, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
            vector<RotatedRect> minRect2( contours2.size() );
            for( int i = 0; i< contours2.size(); i++ ){
                minRect2[i] = minAreaRect( Mat(contours2[i]) );
                double resultSize = 0.0;
                if(minRect2[i].angle < 50 && minRect2[i].angle > -50)
                    resultSize = minRect2[i].size.height;
                else
                    resultSize = minRect2[i].size.width;
                
                if(20 < resultSize){
                    resultSize = resultSize/(m_points[2].y-m_points[0].y)*realSize;// posledny
                    resultSize = resultSize / 10;
                    char buffer[32];
                    memset(buffer, 0, sizeof(buffer));
                    snprintf(buffer, sizeof(buffer), "%.1f cm", resultSize);
                    putText(f_resized,buffer
                            /*to_string((int) minRect2[i].size.height)*/,
                            Point(minRect2[i].center.x-15+o_best.x,minRect2[i].center.y+o_best.y),
                            FONT_HERSHEY_PLAIN, 1, Scalar(0,50,255), 2);
                    cv::rectangle(f_resized, minRect2[i].boundingRect(), Scalar(50,50,100));
                    cout<<minRect2[i].size.height<<" : "<<minRect2[i].boundingRect().height<<endl;
                }
            }
        
            cv::rectangle(f_resized, o_best, Scalar(255,0,255));
            for(int m = 0; m < m_points.size(); m++){
                cv::Point pointCenter(o_best.x+m_points[m].x,o_best.y+m_points[m].y);
                cv::circle(f_resized,pointCenter,2, Scalar(0,255,255));
            }            
        }
        
        tm.stop();
        std::cout << "ID:\t" << f_id << "\t"
//                << "Contours:\t" << contours.size() << "\t"
//                << "Count:\t" << count << "\t"
                << "Time:\t" << tm.getTimeMilli() << "ms" << std::endl;
        cv::imshow("src", f_resized/*Mat(Size(100,100), CV_8UC3)*/);
        if(!f_sectors.empty())
            cv::imshow("HSV Treshold", f_sectors);
        f_id++;
        if(cv::waitKey(1) == 27)
            return EXIT_SUCCESS;
    }
    
//    delete sem;

    return 0;
}
/*ofstream myfile;
    myfile.open ("example.csv",ios::out | ios::app);
    myfile << "Writing this to a file.\n";
    myfile << minRect[i].center.x << ';' << minRect[i].center.y<<';';*/
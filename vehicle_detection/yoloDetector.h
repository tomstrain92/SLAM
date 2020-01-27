#ifndef VEHICLEDETECTION_H
#define VEHICLEDETECTION_H

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"

#include "opencv2/imgproc/imgproc_c.h"

#include <fstream>
#include <sstream>
#include <iostream>

using namespace cv;
using namespace dnn;
using namespace std;


class Detector{

public: 
    Net net;
    string model;
    // Initialize the parameters globals
    float confThreshold = 0.5; // Confidence threshold
    float nmsThreshold = 0.4;  // Non-maximum suppression threshold
    int inpWidth = 416;
    // Width of network's input image
    int inpHeight = 416; // Height of network's input image
    vector<string> classes;
  
    Detector(string model);

    void detectAndDisplay(Mat frame, bool show, vector<Rect>& boxesProcessed, vector<string>& labelsProcessed,
        vector<float>& confidencesProcessed);

    void postprocess(Mat& frame, const vector<Mat>& outs, bool& show, 
    vector<Rect>& boxesProcessed, vector<string>& labelsProcessed, vector<float>& confidencesProcessed);
    
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
    
    vector<String> getOutputsNames(const Net& net);

};

#endif 

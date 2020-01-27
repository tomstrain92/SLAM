#ifndef YOLODETECTORGPU_H
#define YOLODETECTORGPU_H

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"

#include <python3.5/Python.h>
#include <stdlib.h>
#include <assert.h>
#include <numpy/arrayobject.h>
#include <numpy/ndarrayobject.h>


// loads detector
PyObject* loadDetector();

// performs the inference
std::vector<cv::Rect> runInference(PyObject *pNet, const cv::Mat frame);


#endif // TRACKING_H

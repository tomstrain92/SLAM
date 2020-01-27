#include "yoloDetectorGPU.h"


int main(int argc, char** argv){

	//setenv("PYTHONPATH", ".", 1);
	const cv::Mat frame = cv::imread(argv[1]);
   
    Py_Initialize();
  	/* this takes care of the memory allocation with py arrays. 
  	Was getting a segementation before using this*/

    
    PyObject *pNet = loadDetector();
    std::vector<cv::Rect_<int>> boxes;
    boxes = runInference(pNet,frame);

    std::cout << boxes.size() << std::endl;

    // draw boxes
    for (int j = 0; j < boxes.size(); j++){   
        std::cout << boxes[j] << std::endl;
        // draw rect on RGB image...
  	    rectangle(frame, boxes[j], cv::Scalar(0,0,0), 2);
    }

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window", frame);
    cv::waitKey(0);

	return 0;
}
#include "yoloDetector.h"

using namespace cv;
using namespace dnn;
using namespace std;


Detector::Detector(string model){
    cout << "\n initialising Detector \n" << endl;
    // Load names of classes
    string classesFile = "/home/tom/Projects/SLAM/vehicle_detection/cfg/coco.names";
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    String modelWeights;
    String modelConfiguration;
     
    // Give the configuration and weight files for the model
    if (model == "yolov3-tiny"){
        modelConfiguration = "~/Projects/SLAM/vehicle_detection/cfg/yolov3-tiny.cfg";
        modelWeights = "~/Projects/SLAM/vehicle_detection/cfg/yolov3-tiny.weights";
    } else {
        modelConfiguration = "/home/tom/Projects/SLAM/vehicle_detection/cfg/yolov3.cfg";
        modelWeights = "/home/tom/Projects/SLAM/vehicle_detection/cfg/yolov3.weights";
    } 

    // Load the network
    net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_OPENCL);
}



void Detector::detectAndDisplay(Mat frame, bool show, 
    vector<Rect>& boxesProcessed, vector<string>& labelsProcessed, vector<float>& confidencesProcessed){
    // Create a 4D blob from a frame.
    Mat blob;
    blobFromImage(frame, blob, 1/255.0, cvSize(inpWidth, inpHeight), Scalar(0,0,0), true, false);

    //Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output of the output layers
    vector<Mat> outs;
    net.forward(outs, getOutputsNames(net));

    //vector<string> labelsProcessed;
    postprocess(frame, outs, show, boxesProcessed, labelsProcessed, confidencesProcessed);

     // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
    if(show)
    {
        vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        string label = format("Inference time for a frame : %.2f ms", t);
        putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

        // show result?!
        imshow("Display", frame);
        waitKey(0);
    }
}


// Remove the bounding boxes with low confidence using non-maxima suppression
void Detector::postprocess(Mat& frame, const vector<Mat>& outs, bool& show, 
	vector<Rect>& boxesProcessed, vector<string>& labelsProcessed, vector<float>& confidencesProcessed){
    
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }
    
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    // this vector will be returned
  	//vector<Rect> boxesProcessed;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        if (show)
        {
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
        }
        boxesProcessed.push_back(boxes[idx]);
        labelsProcessed.push_back(classes[classIds[idx]]);
        confidencesProcessed.push_back(confidences[idx]);
    }
}


// Draw the predicted bounding box
void Detector::drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame){
    
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);
    
    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }
    
    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);
}



// Get the names of the output layers
vector<String> Detector::getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();
        
        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();
        
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}


// main
int main(int argc, char** argv)
{

    std::cout << "Using OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;
    Mat image = imread("../cars.jpg");
    Mat frame;
    cv::resize(image, frame, cv::Size(), 0.5, 0.5);

    // Check for failure
    if (image.empty()) 
    {
    cout << "Could not open or find the image" << endl;
    cin.get(); //wait for any key press
    return -1;
    }

    // performing detection ...
    string model = "yolov3";
    Detector* detector;
    detector = new Detector(model);

    bool show = true;
    vector<Rect> boxes;
    vector<string> labels;
    vector<float> confidences; 
    detector->detectAndDisplay(frame, show, boxes, labels, confidences);

    for (int i = 0; i < boxes.size(); i++)
    {
        cout << "box: " << boxes[i] << ", label: " << labels[i] << endl;
        cout<< "confidence: " << confidences[i] << endl;
    }

    return 0;
}

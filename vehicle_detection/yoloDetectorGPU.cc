#include "yoloDetectorGPU.h"

using namespace cv;
using namespace dnn;
using namespace std;


PyObject* loadDetector(){

	cout << "in loadDetector" << endl;

	PyObject *pName, *pMod, *pPyDetector, *pArgs, *pNet;

	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.insert(0, '/home/tom/Projects/SLAM/vehicle_detection/')");

	cout << "about to import " << endl;
    pName = PyUnicode_DecodeFSDefault("yoloDetectorGPU");
    cout << "imported module" << endl;
    pMod = PyImport_Import(pName);
   	Py_DECREF(pName);

	cout << "imported module" << endl;

   	// loading the detector as a pyObject - this will be passed in the code instead 
   	// of detector class in code currently. 

   	pPyDetector = PyObject_GetAttrString(pMod, "load_detector");
   	Py_DECREF(pMod);

   	if (pPyDetector && PyCallable_Check(pPyDetector)){

   		pArgs = PyTuple_New(0);
   		pNet = PyObject_CallObject(pPyDetector, pArgs);
   		Py_DECREF(pArgs);

	}
	Py_DECREF(pPyDetector);

   	return pNet;

}


std::vector<cv::Rect> runInference(PyObject *pNet, const cv::Mat frame){

	// now creating open cv rect objects
   	vector<Rect_<int>> vehicle_boxes;

	_import_array();

	PyObject *pName, *pMod, *pArgs, *pPyInference, *pImg, *pPyBoxes;
	PyArrayObject *pArrBoxes;

	pName = PyUnicode_DecodeFSDefault("yoloDetectorGPU");
    pMod = PyImport_Import(pName);
    Py_DECREF(pName);

    pPyInference = PyObject_GetAttrString(pMod, "run_inference");
    Py_DECREF(pMod);

   	if (pPyInference && PyCallable_Check(pPyInference)){

   		pArgs = PyTuple_New(2);
   		PyTuple_SetItem(pArgs, 0, pNet);
   		
   		npy_intp dimensions[3] = {frame.rows, frame.cols, frame.channels()};
   		pImg = PyArray_SimpleNewFromData(frame.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, frame.data);
   		PyTuple_SetItem(pArgs, 1, pImg);
   		
  		pPyBoxes = PyObject_CallObject(pPyInference, pArgs);

   		pArrBoxes = reinterpret_cast<PyArrayObject*>(pPyBoxes);
   		npy_intp pnVehicles = PyArray_DIM(pArrBoxes, 0) / 4;
   		
   		long int *pBoxes = reinterpret_cast<long int*>(PyArray_DATA(pArrBoxes));

   		for (npy_intp i=0; i<pnVehicles; i++){

   			int top = pBoxes[4*i + 0];
   			int left = pBoxes[4*i + 1];
   			int width = pBoxes[4*i + 2];
   			int height = pBoxes[4*i + 3];

   			Rect_<int> vehicle_box = Rect(top, left, width, height);
   			vehicle_boxes.push_back(vehicle_box);
   		}

   	}
   	//cout << vehicle_boxes << endl;
   	return vehicle_boxes;
   	
}

#include <estimateTransformation.h>
#include "Converter.h"
#include "math.h"

#define PI 3.14159265

using namespace std;

namespace plt = matplotlibcpp;


PyArrayObject* create_1D_pyArray(std::vector<double> array1D, int len)
{
	PyObject* pyArray; // to return

	npy_intp dims[1] = {len};

	PyArrayObject* vec_array = (PyArrayObject *) PyArray_SimpleNew(1, dims, PyArray_DOUBLE);
	double *vec_array_pointer = (double*) PyArray_DATA(vec_array);

	copy(array1D.begin(),array1D.end(),vec_array_pointer);

	return vec_array;

}


// THIS IS AMAZING FOUND HERE: https://stackoverflow.com/questions/18780570/passing-a-c-stdvector-to-numpy-array-in-python
PyArrayObject* create_2D_pyArray(std::vector<std::vector<double>> array2D)
{
	/* this function loads a py array object from a vector of vector of doubles
	WARNING each vector must be the same length ! */
	PyObject* pyArray; // to return
	int nRows = array2D.size();
	int nCols = array2D[0].size();

	npy_intp dimensions[2] = {nRows, nCols};

	PyArrayObject* vec_array = (PyArrayObject *) PyArray_SimpleNew(2, dimensions, PyArray_DOUBLE);

	double *vec_array_pointer = (double*) PyArray_DATA(vec_array);

	for (size_t iRow=0; iRow < nRows; ++iRow)
	{
		copy(array2D[iRow].begin(),array2D[iRow].end(),vec_array_pointer+iRow*nCols);
    }

    return vec_array;

}


std::vector<double> runEstimation(const std::vector<std::vector<double>> x_GPS, const std::vector<std::vector<double>> x_SLAM,
					std::vector<double> params0)
{
	Py_Initialize();
	_import_array();
	// loading python scripts
	PyObject *pName, *pMod, *pPyEstimator, *pArgs, *pX_GPS, *pX_SLAM, *pParams;

	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.insert(0, '/home/tom/Projects/SLAM/coordinate_transformation/')");

	//std::cout << "about to import " << std::endl;
    pName = PyUnicode_DecodeFSDefault("estimate_transformation");
    //std::cout << "imported module" << std::endl;
    pMod = PyImport_Import(pName);
   	Py_DECREF(pName);

	//std::cout << "imported module" << std::endl;

   	// loading the detector as a pyObject - this will be passed in the code instead
   	// of detector class in code currently.
	// create vector to return
	std::vector<double> transform;

   	pPyEstimator = PyObject_GetAttrString(pMod, "estimate_transformation");
   	Py_DECREF(pMod);

   	if (pPyEstimator && PyCallable_Check(pPyEstimator)){
		//std::cout << "callable" << '\n';
		// create pyArrays
		pX_GPS = (PyObject*) create_2D_pyArray(x_GPS);
		pX_SLAM = (PyObject*) create_2D_pyArray(x_SLAM);
		pParams = (PyObject*) create_1D_pyArray(params0, 7);

   		// create arguments and pass to estimator
		pArgs = PyTuple_New(3);
		PyTuple_SetItem(pArgs, 0, pX_GPS);
		PyTuple_SetItem(pArgs, 1, pX_SLAM);
		PyTuple_SetItem(pArgs, 2, pParams);
#define WITH_OPENCV

		PyObject* pPyTransform = PyObject_CallObject(pPyEstimator, pArgs);

		PyArrayObject *pArrTransform = reinterpret_cast<PyArrayObject*>(pPyTransform);
		double *pTransform = reinterpret_cast<double*>(PyArray_DATA(pArrTransform));

		for (npy_intp i = 0; i<7; i++)
		{
			transform.push_back(pTransform[i]);
		}
   		Py_DECREF(pArgs);

	}
	Py_DECREF(pPyEstimator);

	return transform;

}
#define WITH_OPENCV


std::vector<std::vector<float>> projectAssets(std::vector<std::vector<double>> asset_coords,
	 cv::Mat camera_rotation, cv::Mat camera_translation, cv::Mat Tcw)
{
	// camera parameters
	float fx = 1029.0;
	float fy = 1133.0;
	float cx = 640.0;
	float cy = 512.0;
	float m = 2 * cx;
	float n = 2 * cy;

	// get rotations
	float R11 = camera_rotation.at<float>(0,0);
	float R12 = camera_rotation.at<float>(0,1);
	float R13 = camera_rotation.at<float>(0,2);

	float R21 = camera_rotation.at<float>(1,0);
	float R22 = camera_rotation.at<float>(1,1);
	float R23 = camera_rotation.at<float>(1,2);

	float R31 = camera_rotation.at<float>(2,0);
	float R32 = camera_rotation.at<float>(2,1);
	float R33 = camera_rotation.at<float>(2,2);

	// and translations
	float t1 = camera_translation.at<float>(0);
	float t2 = camera_translation.at<float>(1);
	float t3 = camera_translation.at<float>(2);

	// Tcw matrix..
	float T11 = camera_rotation.at<float>(0,0);
	float T12 = camera_rotation.at<float>(0,1);
	float T13 = camera_rotation.at<float>(0,2);
	float T14 = camera_rotation.at<float>(0,3);

	float T21 = camera_rotation.at<float>(1,0);
	float T22 = camera_rotation.at<float>(1,1);
	float T23 = camera_rotation.at<float>(1,2);
	float T24 = camera_rotation.at<float>(0,3);

	float T31 = camera_rotation.at<float>(2,0);
	float T32 = camera_rotation.at<float>(2,1);
	float T33 = camera_rotation.at<float>(2,2);
	float T34 = camera_rotation.at<float>(2,3);

	float T41 = camera_rotation.at<float>(3,0);
	float T42 = camera_rotation.at<float>(3,1);
	float T43 = camera_rotation.at<float>(3,2);
	float T44 = camera_rotation.at<float>(3,3);

	std::cout << "camera slam position: [" << t1 << ", " << t2 << ", " << t3 << "]" <<'\n';

	// points to return.
	std::vector<std::vector<float>> asset_points;
	// convert each of the asset_coords
	int nAssets = asset_coords.size();
	for(int i = 0; i < nAssets; i++)
	{

		// translate in the frame of the camera in SLAM world
		float x_asset_SLAM = (float) asset_coords[i][0];
		float y_asset_SLAM = (float) asset_coords[i][1];
		float z_asset_SLAM = (float) asset_coords[i][2];
		//std::cout << "asset slam position: [" << x_asset_SLAM << ", " << y_asset_SLAM << ", " << z_asset_SLAM<< "]" <<'\n';


		float x_Tcw = (T11 * x_asset_SLAM) + (T12 * y_asset_SLAM) + (T13 * z_asset_SLAM) + T14;
		float y_Tcw = (T21 * x_asset_SLAM) + (T22 * y_asset_SLAM) + (T23 * z_asset_SLAM) + T24;
		float z_Tcw = (T31 * x_asset_SLAM) + (T32 * y_asset_SLAM) + (T33 * z_asset_SLAM) + T34;
		//std::cout << "computed by Tcw: " << x_Tcw << ", " << y_Tcw << ", " << z_Tcw << "]" << '\n';

		// rotate:
		float x_cam = (R11 * (x_asset_SLAM - t1)) + (R12 * (y_asset_SLAM - t2)) + (R13 * (z_asset_SLAM - t3));
		float y_cam = (R21 * (x_asset_SLAM - t1)) + (R22 * (y_asset_SLAM - t2)) + (R23 * (z_asset_SLAM - t3));
		float z_cam = (R31 * (x_asset_SLAM - t1)) + (R32 * (y_asset_SLAM - t2)) + (R33 * (z_asset_SLAM - t3));

		//std::cout << "computed by R and t : " << x_cam << ", " << y_cam << ", " << z_cam << "]" << '\n';

		// project:
		float u = (fx * (x_Tcw / z_Tcw)) + cx;
		float v = (fy * (y_Tcw / z_Tcw)) + cy;
		// if in image create point and add to vector
		if (u > 0.0 && u < m && v > 0.0 && v < n)
		{
			//std::cout << "pixels: [" << u << ", " << v << "]" << '\n';
			std::vector<float> point = {u,v};
			asset_points.push_back(point);
		}
	}

	return asset_points;
}


std::vector<double> transformGPSCoordinate2SLAM(std::vector<double> transform, std::vector<double> x_GPS)
{
	Py_Initialize();
	_import_array();
	// loading python scripts
	PyObject *pName, *pMod, *pPyTransformation, *pArgs, *pX_GPS, *pTransform;

	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.insert(0, '/home/tom/Projects/SLAM/coordinate_transformation/')");

	//std::cout << "about to import " << std::endl;
    pName = PyUnicode_DecodeFSDefault("estimate_transformation");
    //std::cout << "imported module" #define WITH_OPENCV
    pMod = PyImport_Import(pName);
   	Py_DECREF(pName);

	//std::cout << "imported module" << std::endl;

	// create vector to return
	std::vector<double> x_slam;

   	pPyTransformation = PyObject_GetAttrString(pMod, "inverse_transform_coordinate");
   	Py_DECREF(pMod);

   	if (pPyTransformation && PyCallable_Check(pPyTransformation)){
		//std::cout << "callable" << '\n';
		// create pyArrays
		pX_GPS = (PyObject*) create_1D_pyArray(x_GPS, 3);
		pTransform = (PyObject*) create_1D_pyArray(transform, 7);

   		// create arguments and pass to transformation functino
		pArgs = PyTuple_New(2);
		PyTuple_SetItem(pArgs, 0, pTransform);
		PyTuple_SetItem(pArgs, 1, pX_GPS);

		PyObject* pPyX_SLAM = PyObject_CallObject(pPyTransformation, pArgs);

		PyArrayObject *pArrX_SLAM= reinterpret_cast<PyArrayObject*>(pPyX_SLAM);
		double *pX_SLAM = reinterpret_cast<double*>(PyArray_DATA(pArrX_SLAM));

		for (npy_intp i = 0; i<3; i++)
		{
			//std::cout << pX_SLAM[i] << '\n';
			x_slam.push_back(pX_SLAM[i]);
		}
   		Py_DECREF(pArgs);

	}
	Py_DECREF(pPyTransformation);

	return x_slam;
}


void plotAssetsAndCamera(std::vector<std::vector<double>> asset_coords_GPS,
	std::vector<std::vector<double>> camera_coords_GPS,
	std::vector<std::vector<double>> asset_coords_SLAM,
	std::vector<std::vector<double>> camera_coords_SLAM,
	cv::Mat imRGB, std::vector<std::vector<float>> asset_points,
	std::vector<float> euler_angles)
{

	// unpacking camera coordinates.
	std::vector<double> cam_x_gps, cam_y_gps, cam_z_gps;
	std::vector<double> cam_x_slam, cam_y_slam, cam_z_slam;

	for (int i = 0; i < camera_coords_GPS.size(); i++)
	{
		cam_x_gps.push_back(camera_coords_GPS[i][0]);
		cam_y_gps.push_back(camera_coords_GPS[i][1]);
		cam_z_gps.push_back(camera_coords_GPS[i][2]);
		cam_x_slam.push_back(camera_coords_SLAM[i][0]);
		cam_y_slam.push_back(camera_coords_SLAM[i][1]);
		cam_z_slam.push_back(camera_coords_SLAM[i][2]);
	}
	// unpacking asset coordinates.
	std::vector<double> asset_x_gps, asset_y_gps, asset_z_gps;
	std::vector<double> asset_x_slam, asset_y_slam, asset_z_slam;

	for (int i = 0; i < asset_coords_GPS.size(); i++)
	{
		asset_x_gps.push_back(asset_coords_GPS[i][0]);
		asset_y_gps.push_back(asset_coords_GPS[i][1]);
		asset_z_gps.push_back(asset_coords_GPS[i][2]);
		asset_x_slam.push_back(asset_coords_SLAM[i][0]);
		asset_y_slam.push_back(asset_coords_SLAM[i][1]);
		asset_z_slam.push_back(asset_coords_SLAM[i][2]);
	}
	// unpacking asset pixels
	std::vector<float> asset_u, asset_v;
	for (int i=0; i < asset_points.size(); i++)
	{
		asset_u.push_back(asset_points[i][0]);
		asset_v.push_back(asset_points[i][1]);
	}
	//
	// // plotting
	// plt::suptitle("Camera Coordinates");
	// plt::subplot(2,2,1);
	// plt::plot(cam_x_gps, cam_y_gps, "o");
	// plt::plot(asset_x_gps, asset_y_gps, "k+");
	// plt::title("GPS");
	// plt::axis("equal");
	// plt::xlabel("Easting");
	// plt::ylabel("Northing");

	plt::subplot(2,2,1);
	plt::plot(cam_x_slam, cam_y_slam, "o");
	//plt::plot(asset_x_slam, asset_y_slam, "k+");

	plt::axis("equal");
	plt::xlabel("SLAM x coordinate");
	plt::ylabel("SLAM y coordinate");

	// plotting heading too.
	double roll = euler_angles[0];
	vector<vector<double>> arm_plot = createAngleArms(cam_x_slam, cam_y_slam, roll);
	plt::plot(arm_plot[0], arm_plot[1]);

	// plotting x,z. should be just zeros. (roll)

	plt::subplot(2,2,2);
	plt::plot(cam_x_slam, cam_z_slam, "o");
	//plt::plot(asset_x_slam, asset_z_slam, "k+");

	plt::axis("equal");
	plt::xlabel("SLAM x coordinate");
	plt::ylabel("SLAM z coordinate");

	double pitch = euler_angles[1];
	vector<vector<double>> arm_plot_2 = createAngleArms(cam_x_slam, cam_z_slam, pitch);
	plt::plot(arm_plot_2[0], arm_plot_2[1]);


	plt::subplot(2,2,3);
	plt::plot(cam_y_slam, cam_z_slam, "o");
	//plt::plot(asset_y_slam, asset_z_slam, "k+");

	plt::axis("equal");
	plt::xlabel("SLAM y coordinate");
	plt::ylabel("SLAM z coordinate");

	double yaw = euler_angles[2];
	vector<vector<double>> arm_plot_3 = createAngleArms(cam_y_slam, cam_z_slam, yaw);
	plt::plot(arm_plot_3[0], arm_plot_3[1]);

	// plotting image
	plt::subplot(2,2,4);
	plt::imshow(imRGB.data, imRGB.rows, imRGB.cols, imRGB.channels());


	std::vector<std::vector<double>> x = {cam_x_slam};
	std::vector<std::vector<double>> y = {cam_y_slam};
	std::vector<std::vector<double>> z = {cam_z_slam};

	//estimating yaw information...
	double zdiff = cam_z_slam[asset_points.size()-1] - cam_z_slam[0];
	double ydiff = cam_y_slam[asset_points.size()-1] - cam_z_slam[0];
	double xdiff = cam_x_slam[asset_points.size()-1] - cam_x_slam[0];

	double angy1 = atan2(xdiff, zdiff) * PI / 180;
	double angy2 = atan2(zdiff, xdiff) * PI / 180;

	double angx1 = atan2(ydiff, zdiff) * PI / 180;
	double angx2 = atan2(zdiff, ydiff) * PI / 180;

	float angz1 = atan2(ydiff, xdiff) * PI / 180;
	float angz2 = atan2(xdiff, ydiff) * PI / 180;

	std::cout << "euler: [" << euler_angles[0] * 180 / PI << ", " << euler_angles[1] * 180 / PI << ", " << euler_angles[2] * 180 / PI << "]" <<'\n';
	std::cout << "x: [" << angx1 << ", " << angx2 << "," << "]" <<'\n';
	std::cout << "y: [" << angy1 << ", " << angy2 << "," << "]" <<'\n';
	std::cout << "z: [" << angz1 << ", " << angz2 << "," << "]" <<'\n';
	//plt::plot(asset_u, asset_v,"ro");


	//
	if (asset_u.size() > 0)
	{
	 	plt::pause(4);

	}else{
		plt::pause(0.5);
	}
	plt::close();
	//plt::clf();


}



std::vector<std::vector<double>> createAngleArms(std::vector<double> coords1,
	std::vector<double> coords2, double angle)
{
	double coord1_last = coords1[coords1.size()-1];
	double coord2_last = coords2[coords2.size()-1];

	double coord1_dist = coord1_last - coords1[coords1.size()-2];
	double coord2_dist = coord2_last - coords2[coords2.size()-2];

	double arm_length = sqrt((coord1_dist * coord1_dist) + (coord2_dist * coord2_dist)) / 2;

	//cout << arm_length << endl;

	double coord1_arm = coord1_last + sin(angle) * arm_length;
	double coord2_arm = coord2_last + cos(angle) * arm_length;

	std::vector<std::vector<double>> coords_to_plot = {{coord1_last, coord1_arm}, {coord2_last, coord2_arm}};
	return coords_to_plot;
}

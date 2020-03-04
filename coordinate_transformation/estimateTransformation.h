#ifndef ESTIMATETRANSFORMATION_H
#define ESTIMATETRANSFORMATION_H

#include <python3.5/Python.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <numpy/arrayobject.h>
#include <numpy/ndarrayobject.h>
#include <iostream>
#include <vector>
#include<opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>

#include <matplotlib-cpp/matplotlibcpp.h>



PyArrayObject* create_1D_pyArray(std::vector<double>, int len);

PyArrayObject* create_2D_pyArray(std::vector<std::vector<double>> array2D);

std::vector<double> runEstimation(const std::vector<std::vector<double>> x_GPS, const std::vector<std::vector<double>> x_SLAM,
					std::vector<double> params0);

std::vector<std::vector<float>> projectAssets(std::vector<std::vector<double>> asset_coords,
	 cv::Mat camera_rotation, cv::Mat camera_translation, cv::Mat Tcw);

std::vector<double> transformGPSCoordinate2SLAM(std::vector<double> transform, std::vector<double> x_GPS);

void plotAssetsAndCamera(std::vector<std::vector<double>> asset_coords_GPS,
	std::vector<std::vector<double>> camera_coords_GPS,
	std::vector<std::vector<double>> asset_coords_SLAM,
	std::vector<std::vector<double>> camera_coords_SLAM,
 	cv::Mat imRGB, std::vector<std::vector<float>> asset_points,
	std::vector<float> euler_angles);

std::vector<std::vector<double>> createAngleArms(std::vector<double> coords1,
	std::vector<double> coords2, double angle);


#endif

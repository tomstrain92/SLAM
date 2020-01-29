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
#include <math.h>

PyArrayObject* create_1D_pyArray(std::vector<double>, int len);

PyArrayObject* create_2D_pyArray(std::vector<std::vector<double>> array2D);

std::vector<double> runEstimation(const std::vector<std::vector<double>> x_GPS, const std::vector<std::vector<double>> x_SLAM,
					std::vector<double> params0);

void projectAssets(std::vector<double> transform, std::vector<std::vector<double>> asset_coords, cv::Mat cameraMatrix,
					cv::Mat cameraTranslation, cv::Mat cameraRotation);

std::vector<double> transformGPSCoordinate2SLAM(std::vector<double> transform, std::vector<double> x_GPS);

#endif

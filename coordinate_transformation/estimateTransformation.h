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


PyObject* create_1D_pyArray(double *array1D, int len);

PyArrayObject* create_2D_pyArray(std::vector<std::vector<double>> array2D);

std::vector<double> runEstimation(const std::vector<std::vector<double>> x_GPS, const std::vector<std::vector<double>> x_SLAM,
					double *params0);

#endif

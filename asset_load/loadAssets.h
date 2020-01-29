#include <python3.5/Python.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <numpy/arrayobject.h>
#include <numpy/ndarrayobject.h>
#include <iostream>
#include <vector>

PyObject* create_1D_pyArray_for_GPS(double *array1D, int len);

std::vector<std::vector<double>> loadAssets(std::vector<double> vdGPS_coords);

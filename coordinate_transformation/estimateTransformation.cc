#include <estimateTransformation.h>

using namespace std;


PyObject* create_1D_pyArray(double *array1D, int len)
{
	PyObject* pyArray; // to return

	npy_intp dimensions[1] = {len};
	pyArray = PyArray_SimpleNewFromData(1, (npy_intp*)&dimensions, NPY_DOUBLE, array1D);

	return pyArray;
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
					double *params0)
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
		pParams = create_1D_pyArray(params0, 7);

   		// create arguments and pass to estimator
		pArgs = PyTuple_New(3);
		PyTuple_SetItem(pArgs, 0, pX_GPS);
		PyTuple_SetItem(pArgs, 1, pX_SLAM);
		PyTuple_SetItem(pArgs, 2, pParams);

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

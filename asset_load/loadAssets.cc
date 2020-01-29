#include <loadAssets.h>


PyObject* create_1D_pyArray_for_GPS(double *array1D, int len)
{
	PyObject* pyArray; // to return

	npy_intp dimensions[1] = {len};
	pyArray = PyArray_SimpleNewFromData(1, (npy_intp*)&dimensions, NPY_DOUBLE, array1D);

	return pyArray;
}


std::vector<std::vector<double>> loadAssets(std::vector<double> vdGPS_coords)
{

	Py_Initialize();
	_import_array();
	// loading python scripts
	PyObject *pName, *pMod, *pAssetLoader, *pArgs, *pGPS_coords;

	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.insert(0, '/home/tom/Projects/SLAM/asset_load/')");

	//std::cout << "about to import " << std::endl;
    pName = PyUnicode_DecodeFSDefault("load_assets");
    pMod = PyImport_Import(pName);
	//std::cout << "imported module" << std::endl;
   	Py_DECREF(pName);

   	pAssetLoader = PyObject_GetAttrString(pMod, "load_assets_from_cpp");
   	Py_DECREF(pMod);

	// vector of vectors that will be returned.
	std::vector<std::vector<double>> asset_coords;

   	if (pAssetLoader && PyCallable_Check(pAssetLoader)){

		double GPS_coords [2] = {vdGPS_coords[0], vdGPS_coords[1]};
		pGPS_coords = create_1D_pyArray_for_GPS(GPS_coords,2);

		pArgs = PyTuple_New(1);
		PyTuple_SetItem(pArgs, 0, pGPS_coords);

		PyObject* pPyAssets = PyObject_CallObject(pAssetLoader, pArgs);
		PyArrayObject *pArrAssets = reinterpret_cast<PyArrayObject*>(pPyAssets);
		// getting number of assets from array length
		npy_intp pNAssets = PyArray_DIM(pArrAssets, 0) / 3;
		// recasting to double.
		double *pAssets = reinterpret_cast<double*>(PyArray_DATA(pArrAssets));
		//std::cout << pAssets << '\n';

		for (npy_intp i = 0; i<pNAssets; i++)
		{
			double asset_x = pAssets[3 * i];
			double asset_y = pAssets[3 * i + 1];
			double asset_z = pAssets[3 * i + 2];
			//std::cout << asset_x << ", " << asset_y << ", " << asset_z << '\n';
			asset_coords.push_back({asset_x, asset_y, asset_z});
		}
		Py_DECREF(pArgs);
	}
	Py_DECREF(pAssetLoader);

	return asset_coords;
}

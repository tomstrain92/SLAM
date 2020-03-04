#include <estimateTransformation.h>

using namespace std;

int main(int argc, char** argv){

	// creating the same dummy datahttps://twitter.com/
	std::vector<std::vector<double>> x_SLAM, x_GPS;

	x_GPS = {{400008.4302111,  100030.64124689,    -11.98923232},{400015.24250915, 100027.79832605,    -20.38627921},
    {400017.35560045, 100030.06409552,    -16.46185137},{399992.28577982, 100064.50316682,     40.85703261},
    {400004.8317672,  100036.49454666,     24.79106245},{400020.50317437, 100012.27285597,     -7.88487208},
    {399977.85047869, 100028.29431596,     -5.63534526},{399990.48468184, 100102.15249591,    -43.61357805},
    {399964.85971891, 100077.42795125,    -15.54120434},{399907.7948278, 100025.01848058,    78.56961113}};

	x_SLAM = {{1,3,1},{2,3,0.5},{2,3,1},{-3,4,6},{-1,2,4},{2,1,1},{-2,3,0.1},{0.4,11,1},{-3,8,1},{-12,0.2,3.6}};

 	std::vector<double> params_init = {0.1,-10,-25,-1,0,2,-2};
	std::vector<double> params;

	// simulating loop
	for (int iLoop = 0; iLoop < 10; iLoop++)
	{

		params = runEstimation(x_GPS, x_SLAM, params_init);
		params_init = params;

		for (int i = 0; i < 7; i++)
		{
			std::cout << params[i] << '\n';
		}

		std::vector<double> x_slam_1 = transformGPSCoordinate2SLAM(params, x_GPS[0]);
		for (int i = 0; i < 3; i++)
		{
			std::cout << x_slam_1[i] << std::endl;
		}
	}

	std::vector<std::vector<double>> asset_coords_GPS;
	std::vector<std::vector<double>> asset_coords_SLAM;

	cv::Mat imRGB;
	imRGB = cv::imread("../000066.jpg");

	std::vector<std::vector<float>> asset_points;

	plotAssetsAndCamera(asset_coords_GPS, x_GPS, asset_coords_SLAM, x_SLAM, imRGB, asset_points);

}

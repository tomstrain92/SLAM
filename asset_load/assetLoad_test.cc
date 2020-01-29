#include <loadAssets.h>

int main(int argc, char** argv){

	std::vector<double> GPS_coords = {465647.225957, 104605.686189};
	std::vector<std::vector<double>> asset_coords = loadAssets(GPS_coords);

	std::cout << asset_coords.size() << " assets loaded" << '\n';

}

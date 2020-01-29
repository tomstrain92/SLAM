import sys
sys.path.insert(0, '/home/tom/Projects/Utils/')
from assets import load_assets
import numpy as np


def load_assets_from_cpp(GPS_coords):
	""" returns a 3n x 1 array of asset coordinates in the world. The dimensions
	are for simplicity with integration with c++. The list MUST be a numpy array."""
	assets = load_assets("A27", "SNSF", XCOORD=GPS_coords[0], YCOORD=GPS_coords[1], max_dist=50)
	asset_list = []
	for (ind, asset) in assets.iterrows():
		asset_list.extend([asset['XCOORD'], asset['YCOORD'], asset['MOUNTING_H']])
	# must return assets a numpy array
	return np.array(asset_list)

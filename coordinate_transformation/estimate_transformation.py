from scipy.optimize import least_squares
import numpy as np
from scipy.spatial.transform import Rotation
import scipy


def transform_coordinate(transform, coord):
	""" performs the 7d transformation """

	scale = transform[0]
	R = Rotation.from_euler('zyx', [transform[1],transform[2],transform[3]], degrees=True);
	t = np.array(transform[4:])
	return (scale * R.apply(coord)) + t


def transform_function(params, x_GPS, x_SLAM):
	"""params contains [scale, q, t] to map x_SLAM to x_GPS"""

	x_SLAM_transform = np.empty((0,3))
	# keeping out of loop for speed
	scale = params[0]
	R = Rotation.from_euler('zyx', [params[1],params[2],params[3]], degrees=True);
	t = np.array(params[4:])
	#errors = np.empty((0,3))
	for gps, slam in zip(x_GPS, x_SLAM):
		# perform transformation
		slam_transform = (scale * R.apply(gps)) + t
		# add to numpy nd array
		x_SLAM_transform = np.vstack([x_SLAM_transform, slam_transform])
	# this is the percentage error as 1d array required by least squares
	return ((x_SLAM_transform - x_SLAM)).ravel()


def estimate_transformation(x_GPS, x_SLAM, params0):
	""" takes in the a matrix of n gps coordinates (3 x n) and slam coordinates
	(3 x n) and estimates a transformation between them of the form

	x_SLAM = lambda * R * x_GPS + t """

	bounds = ([0,-180,-180,-180,-np.inf,-np.inf,-np.inf],
			  [10,180,180,180,np.inf,np.inf,np.inf])

	res = least_squares(transform_function, params0, verbose=1, args=(x_GPS,x_SLAM),
			bounds=bounds,ftol=1e-4, xtol=1e-4)

	transformation = res.x
	print("solved with residual {:f}".format(res.cost))

	return transformation

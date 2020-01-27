from scipy.optimize import least_squares
import numpy as np
from scipy.spatial.transform import Rotation


def transform_function(params, x_GPS, x_SLAM):
	"""params contains [scale, q, t] to map x_SLAM to x_GPS"""
	scale = params[0]
	R = Rotation.from_euler('zyx', [params[1],params[2],params[3]], degrees=True);
	t = np.array(params[4:])

	x_SLAM_transform = np.empty((0,3))
	#errors = np.empty((0,3))
	for gps, slam in zip(x_GPS, x_SLAM):
		slam_transform = scale * R.apply(gps) + t
		x_SLAM_transform = np.vstack([x_SLAM_transform, slam_transform])
		#error = np.sqrt(np.sum((gps_transform - gps)**2))
		#errors += error
	return ((x_SLAM_transform - x_SLAM) / (x_SLAM + 0.0005)).ravel()


def estimate_transformation(x_GPS, x_SLAM, params0):
	""" takes in the a matrix of n gps coordinates (3 x n) and slam coordinates
	(3 x n) and estimates a transformation between them of the form

	x_SLAM = lambda * R * x_GPS + t """
	"""
	print("SLAM:")
	print(x_SLAM)
	print("GPS:")
	print(x_GPS)
	print("params")
	print(params0)
	"""

	bounds = ([0,-180,-180,-180,-np.inf,-np.inf,-np.inf],
			  [10,180,180,180,np.inf,np.inf,np.inf])

	res = least_squares(transform_function, params0, verbose=0, args=(x_GPS,x_SLAM),
			bounds=bounds,ftol=1e-4, xtol=1e-4)
	transformation = res.x
	print("solved with residual {:f}".format(res.cost))
	#scale = transformation[0]
	#R = Rotation.from_euler('zyx', [transformation[1],transformation[2],transformation[3]], degrees=True);
	#t = np.array(transformation[4:])
	return transformation

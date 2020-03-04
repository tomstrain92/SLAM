from scipy.optimize import least_squares
import numpy as np
from scipy.spatial.transform import Rotation
import scipy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

##################################################################
#########Code to solve each coordinate individually...############
##################################################################

def linear_coordinate_transform(transform, GPS):

	x_SLAM = transform[0] * GPS[0] + transform[1]
	y_SLAM = transform[2] * GPS[1] + transform[3]
	z_SLAM = transform[4] * GPS[2] + transform[5]

	return np.array([x_SLAM, y_SLAM, z_SLAM])



def linear_transform_function(params, coord_GPS, coord_SLAM):
	scale = params[0]
	offset = params[1]
	error = []
	for gps, slam in zip(coord_GPS, coord_SLAM):
		# perform transformation
		slam_transform = scale * gps + offset
		# add to numpy nd array
		error.append(slam_transform - slam)

	errors = np.array(error).reshape(-1)
	return errors

def estimate_linear_transformation(params, coord_GPS, coord_SLAM):
	""" considering each coordinate individually ..."""
	res = least_squares(linear_transform_function, params, verbose=0, args=(coord_GPS,coord_SLAM),
	ftol=1e-6, xtol=1e-6)

	transformation = res.x
	print("solved with residual {:f}".format(res.cost))
	return transformation

##################################################################
#################### TRANSLATED FUNCTIONS ####################
##################################################################

def transform_translated_coordinate(transform, coord):
	""" this function differs slightly - it assumes both the gps and slam coords have been
	normalised """
	scale = transform[0]
	R = Rotation.from_euler('xyz', [transform[1],transform[2],transform[3]], degrees=True);
	t = transform[4:]
	return  R.apply(scale * (coord + t))


def translated_transform_function(params, x_GPS, x_SLAM):
	"""params contains [scale, q, t] to map x_SLAM to x_GPS"""

	x_SLAM_transform = np.empty((0,3))
	# keeping out of loop for speed
	scale = params[0]
	R = Rotation.from_euler('xyz', [params[1],params[2],params[3]], degrees=True);
	t = params[4:]
	#errors = np.empty((0,3))
	for gps, slam in zip(x_GPS, x_SLAM):
		# perform transformation
		slam_transform = R.apply(scale * (gps + t))
		# add to numpy nd array
		x_SLAM_transform = np.vstack([x_SLAM_transform, slam_transform])
	# this is the percentage error as 1d array required by least squares
	return ((x_SLAM_transform - x_SLAM)).ravel()


def estimate_translated_transformation(x_GPS, x_SLAM, params0):
	""" takes in the a matrix of n gps coordinates (3 x n) and slam coordinates
	(3 x n) and estimates a transformation between them of the form

	x_SLAM = lambda * R * x_GPS"""

	bounds = ([0,-180,-180,-180,-np.inf,-np.inf,-np.inf],
			  [100,180,180,180,np.inf,np.inf,np.inf])

	res = least_squares(translated_transform_function, params0, verbose=1, args=(x_GPS,x_SLAM),
			bounds=bounds,ftol=1e-6, xtol=1e-6)

	transformation = res.x
	print("solved with residual {:f}".format(res.cost))
	print(transformation)
	return transformation

##################################################################
#################### NOT TRANSLATED FUNCTIONS ####################
##################################################################

def inverse_transform_coordinate(transform, coord):
	"""performs the inverse of the 7d transformation"""
	scale = transform[0]
	R = Rotation.from_euler('zyx', [transform[1],transform[2],transform[3]], degrees=True);
	R_inv = R.inv()
	t = np.array(transform[4:])
	return R_inv.apply(((1 / scale) * (coord - t)))


def transform_coordinate(transform, coord):
	""" performs the 7d transformation """

	scale = transform[0]
	R = Rotation.from_euler('zyx', [transform[1],transform[2],transform[3]], degrees=True);
	t = np.array(transform[4:])
	return scale * R.apply(coord) + t


def transform_function(params, x_GPS, x_SLAM):
	"""params contains [scale, q, t] to map x_SLAM to x_GPS"""

	x_GPS_transform = np.empty((0,3))
	# keeping out of loop for speed
	scale = params[0]
	R = Rotation.from_euler('zyx', [params[1],params[2],params[3]], degrees=True);
	t = np.array(params[4:])
	#errors = np.empty((0,3))
	for gps, slam in zip(x_GPS, x_SLAM):
		# perform transformation
		gps_transform = scale * R.apply(slam) + t
		# add to numpy nd array
		x_GPS_transform = np.vstack([x_GPS_transform, gps_transform])
	# this is the percentage error as 1d array required by least squares
	return ((x_GPS_transform - x_GPS)).ravel()


def estimate_transformation(x_GPS, x_SLAM, params0):
	""" takes in the a matrix of n gps coordinates (3 x n) and slam coordinates
	(3 x n) and estimates a transformation between them of the form

	x_GPS = lambda * R * x_SLAM + t """

	bounds = ([0,-180,-180,-180,-np.inf,-np.inf,-np.inf],
			  [100,180,180,180,np.inf,np.inf,np.inf])

	res = least_squares(transform_function, params0, verbose=0, args=(x_GPS,x_SLAM),
			bounds=bounds,ftol=1e-6, xtol=1e-6)

	transformation = res.x
	print("solved with residual {:f}".format(res.cost))
	print(transformation)
	return transformation

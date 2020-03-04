import numpy as np
from estimate_transformation import *
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt


if __name__ == "__main__":

	# transformation
	transform = np.array([20, 10, 25, -30, 400000, -2.3, 100000])
	# creating some dummy data
	mean = [0,0,20]
	cov = [[10,0,0],[0,10,0],[0,0,20]]

	x_SLAM = np.random.multivariate_normal(mean, cov, 20);
	x_GPS = [transform_coordinate(transform,x) for x in x_SLAM]

	x_GPS = np.stack(x_GPS, axis=0)
	print(x_GPS)

	mean = [0,0,0]
	sigma = 0
	cov = [[sigma,0,0],[0,sigma,0],[0,0,sigma]]

	np.set_printoptions(suppress=True)
	print("x_SLAM: ")
	print(x_SLAM)

	# can we now use estimate_transformation to do it.
	init_mean = transform
	init_cov = [[20,0,0,0,0,0,0],
				[0,100,0,0,0,0,0],
				[0,0,100,0,0,0,0],
				[0,0,0,100,0,0,0],
				[0,0,0,0,200000,0,0],
				[0,0,0,0,0,5,0],
				[0,0,0,0,0,0,200000]]

	params0 = np.random.multivariate_normal(init_mean, init_cov, 1);
	params0 = params0.tolist()
	print(params0[0])

	params = estimate_transformation(x_GPS, x_SLAM, params0[0])
	print(params)

	# plotting results
	scale = params[0]
	R = Rotation.from_euler('zyx', [params[1],params[2],params[3]], degrees=True)
	t = np.array(params[4:])

	for ind, slam in enumerate(x_SLAM):
		plt.plot(x_GPS[ind,0], x_GPS[ind,2], 'bo', alpha = 0.2)
		gps_transform = transform_coordinate(params, slam)
		plt.plot(gps_transform[0], gps_transform[2], 'k+')
		plt.title('GPS')


	plt.figure()
	# now mapping the inverse. (from gps to slam)
	for ind, gps in enumerate(x_GPS):
		plt.plot(x_SLAM[ind,0], x_SLAM[ind,2], 'bo', alpha = 0.2)
		slam_transform = inverse_transform_coordinate(params, gps)
		plt.plot(slam_transform[0], slam_transform[2], 'k+')
		plt.title('SLAM')

	plt.show()

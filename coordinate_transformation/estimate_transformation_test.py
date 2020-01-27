import numpy as np
from estimate_transformation import estimate_transformation
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

if __name__ == "__main__":
	# creating some dummy data

	x_GPS = np.array([[400000.112,150000.110,2.301], [400012.123,150300.343,2.322], [400101.124,153010.674,2.123],
					[400049.345,150012.095,2.353], [400232.123,150220.348,1.359], [400155.230,150300.991,3.013],
					[400012.143,150300.456,0.341], [400351.124,150501.064,4.314], [401030.016,150400.123,2.850],
					[400901.862,150018.194,2.378], [401456.739,150600.001,1.684], [400890.596,149821.897,1.031],])

	scale = 0.05;
	R = Rotation.from_euler('zyx', [10,25,-30], degrees=True);
	t = np.array([[-1.5,1,-1]])

	# create corresponding slam data..

	mean = [0,0,0]
	sigma = 0.2
	cov = [[sigma,0,0],[0,sigma,0],[0,0,sigma]]

	x_SLAM = np.empty((0,3))
	for x in x_GPS:
	 	x_SLAM = np.vstack([x_SLAM, scale * R.apply(x) + t]) + np.random.multivariate_normal(mean, cov, 1)

	np.set_printoptions(suppress=True)
	print("x_SLAM: ")
	print(x_SLAM)

	# can we now use estimate_transformation to do it.
	params0 = np.array([0.01,10,10,	100,-0,-1,-3])
	params = estimate_transformation(x_GPS, x_SLAM, params0)
	print(params)

	# plotting results
	scale = params[0]
	R = Rotation.from_euler('zyx', [params[1],params[2],params[3]], degrees=True)
	t = np.array([params[4:]])

	for ind, x in enumerate(x_GPS):
		plt.plot(x_SLAM[ind,0], x_SLAM[ind,1], 'bo', alpha = 0.2)
		slam_transform = scale * R.apply(x) + t
		slam = slam_transform[0]
		plt.plot(slam[0], slam[1], 'k+')

	plt.show()

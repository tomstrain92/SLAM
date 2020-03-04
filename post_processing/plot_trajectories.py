import pandas as pd
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy.spatial.transform import Rotation as R
import numpy as np
from skimage.transform import resize, rescale, downscale_local_mean
from load_data import data_dir
import math

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]



def plotSLAMCAMRoad2D(trajectory, rot0=None):

	for ind, t in trajectory.iterrows():
		plt.figure(1)
		plt.scatter(t[0],t[2],c='tab:red')
		scale = 0.2
		[yaw, pitch, roll] = quaternion_to_euler(t['qx'],t['qy'],t['qz'],t['qw'])
		angle = pitch
		print(np.rad2deg(yaw), np.rad2deg(pitch), np.rad2deg(roll))
		arm = [t[0] + scale*np.sin(angle), 0, t[2] + scale*np.cos(angle)]
		#rot = R.from_euler('xyz',[yaw, -pitch, roll])
		#arm2 = rot.apply([t[0]+1,t[1],t[2]])
		plt.plot([t[0], arm[0]],
				 [t[2], arm[2]], 'k--')


		plt.figure(2)
		angle = roll
		plt.scatter(t[1],t[2],c='tab:red')
		arm = [0, t[1] + scale*np.sin(angle), t[2] + scale*np.cos(angle)]
		plt.plot([t[1], arm[1]],
				 [t[2], arm[2]], 'k--')


def plotSLAMCAMRoad(road_SLAM_CAMs, rot0=None):

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(road_SLAM_CAMs[0,0],road_SLAM_CAMs[0,1],road_SLAM_CAMs[0,2],'r')
	ax.scatter(road_SLAM_CAMs[1:,0],road_SLAM_CAMs[1:,1],road_SLAM_CAMs[1:,2])

	[yaw, pitch, roll] = quaternion_to_euler(rot0['qx'],rot0['qy'],rot0['qz'],rot0['qw'])
	print('yaw: ', np.rad2deg(yaw))
	print('pitch: ', np.rad2deg(pitch))
	print('roll: ', np.rad2deg(roll))

	dx = road_SLAM_CAMs[1,0] - road_SLAM_CAMs[0,0]
	dy = road_SLAM_CAMs[1,1] - road_SLAM_CAMs[0,1]
	dz = road_SLAM_CAMs[1,2] - road_SLAM_CAMs[0,2]
	print('estimated heading: ', (180 / math.pi) * math.atan2(dx, dz))


	ax.set_xlabel('x_cam')
	ax.set_ylabel('z_cam')
	ax.set_zlabel('y_cam')

	X = road_SLAM_CAMs[:,0].ravel()
	Y = road_SLAM_CAMs[:,1].ravel()
	Z = road_SLAM_CAMs[:,2].ravel()

	# Create cubic bounding box to simulate equal aspect ratio
	max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()])
	max_range = max_range.max()
	Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(X.max()+X.min())
	Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(Y.max()+Y.min())
	Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(Z.max()+Z.min())

	# Comment or uncomment following both lines to test the fake bounding box:
	#for xb, yb, zb in zip(Xb, Yb, Zb):
	#   ax.plot([xb], [yb], [zb], 'w')

	if rot0 is not None:
		scale = 0.2
		units = scale * np.array([[0,1,0]])
		t = road_SLAM_CAMs[0]
		rot = R.from_quat([rot0['qx'],rot0['qy'],rot0['qz'],rot0['qw']])
		unit_rot = rot.apply([0,0,1])
		ax.plot([t[0],unit_rot[0]],
				[t[1],unit_rot[1]],
				[t[2],unit_rot[2]])



def plotSLAMRoadPixels(image_file, road_pixels):

	plt.figure()
	img = mpimg.imread(os.path.join(data_dir(),image_file))
	img = rescale(img, 2, anti_aliasing=True)

	plt.imshow(img)
	img = rescale(img, 2, anti_aliasing=True)
	plt.scatter(road_pixels[:,0],road_pixels[:,1])


def plotAssetsOnImage(image_file, asset_pixels_gps=None, asset_pixels_SLAM=None):

	#[path, image_file] = os.path.split(image_file)
	img = mpimg.imread(os.path.join(data_dir(),image_file))
	img = rescale(img, 2, anti_aliasing=True)
	plt.imshow(img)

	if asset_pixels_SLAM is not None:
		print(asset_pixels_SLAM)
		plt.scatter(asset_pixels_SLAM[:,0],asset_pixels_SLAM[:,1])

	if asset_pixels_gps is not None:
		plt.scatter(asset_pixels_gps[:,0],asset_pixels_gps[:,1])


def plotTransformationResult(x_SLAM, x_GPS, assets_SLAM, assets_GPS, transformation):

	fig, ax = plt.subplots(2,2)

	ax[0,0].scatter(x_GPS[:,0], x_SLAM[:,0])
	ax[0,0].scatter(assets_GPS[:,0], assets_SLAM[:,0])
	ax[0,0].set_xlabel('x GPS')
	ax[0,0].set_ylabel('x SLAM')

	# plotting fit
	x_lim = ax[0,0].get_xlim()
	x_plot = np.linspace(x_lim[0], x_lim[1], 10)
	y_plot = transformation[0] * x_plot + transformation[1]
	ax[0,0].plot(x_plot, y_plot, 'k--')


	ax[0,1].scatter(x_GPS[:,1], x_SLAM[:,1])
	ax[0,1].scatter(assets_GPS[:,1], assets_SLAM[:,1])
	ax[0,1].set_xlabel('y GPS')
	ax[0,1].set_ylabel('y SLAM')

	# plotting fit
	x_lim = ax[0,1].get_xlim()
	x_plot = np.linspace(x_lim[0], x_lim[1], 10)
	y_plot = transformation[2] * x_plot + transformation[3]
	ax[0,1].plot(x_plot, y_plot, 'k--')

	ax[1,1].scatter(x_GPS[:,2], x_SLAM[:,2])
	ax[1,1].scatter(assets_GPS[:,2], assets_SLAM[:,2])
	ax[1,1].set_xlabel('z GPS')
	ax[1,1].set_ylabel('z SLAM')

	# plotting fit
	x_lim = ax[1,1].get_xlim()
	x_plot = np.linspace(x_lim[0], x_lim[1], 10)
	y_plot = transformation[4] * x_plot + transformation[5]
	ax[1,1].plot(x_plot, y_plot, 'k--')

	plt.show()


def plotTransformedCoords(x_gps_slam, x_slam, assets=None):

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(x_slam[:,0],x_slam[:,1],x_slam[:,2],'b')
	ax.scatter(x_gps_slam[:,0],x_gps_slam[:,1],x_gps_slam[:,2],'r')

	if assets is not None:
		ax.scatter(assets[:,0], assets[:,2], assets[:,1], marker='^')

	ax.set_xlabel('x SLAM')
	ax.set_ylabel('y SLAM')
	ax.set_zlabel('z SLAM')
	ax.legend(['SLAM','GPS transformed to SLAM'])


def plotGPSAndAssets(GPS, assets=None):

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(GPS[:,0], GPS[:,2], GPS[:,1])

	if assets is not None:
		ax.scatter(assets[:,0], assets[:,2], assets[:,1], marker='^')

	ax.set_xlabel('Easting')

	ax.set_zlabel('Height')
	ax.set_ylabel('Northing')
	ax.set_title('GPS coordinate system')
	#plt.show()


def plotTrajectoryAndAssets(trajectory, assets=None, gps_slam=None, asset_SLAM_CAMs=None):

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	plotTrajectory3D(trajectory, ax)
	#ax.scatter(trajectory[:,0], trajectory[:,1], trajectory[:,2])

	if assets is not None:
		ax.scatter(assets[:,0], assets[:,1], assets[:,2], marker='^')
	if gps_slam is not None:
		ax.scatter(gps_slam[:,0], gps_slam[:,1], gps_slam[:,2], 'r')
	if asset_SLAM_CAMs is not None:
		for asset_SLAM_CAM in asset_SLAM_CAMs:
			ax.plot3D([0,asset_SLAM_CAM[0]],
					  [0,asset_SLAM_CAM[1]],
					  [0,asset_SLAM_CAM[2]],'k--')
	#plt.show()


def plotTrajectory3D(trajectory, ax):
	""" Plots the camera trajectories and their orientations """

	ax.scatter(trajectory['x'], trajectory['y'], trajectory['z'])

	x_scale = abs(trajectory.iloc[1]['x'] - trajectory.iloc[0]['x'])/2
	y_scale = abs(trajectory.iloc[1]['y'] - trajectory.iloc[0]['y'])/2
	z_scale = abs(trajectory.iloc[1]['z'] - trajectory.iloc[0]['z'])/2

	# length of the axes.
	unit_x = [x_scale,0,0]
	unit_y = [0,x_scale,0]
	unit_z = [0,0,x_scale]

	for (ind, frame) in trajectory.iterrows():

		rot = R.from_quat([frame['qx'], frame['qy'], frame['qz'], frame['qw']])
		rot270ZX = R.from_euler('xyz', [270,0,270], degrees=True)
		# drawing rotational frames.
		# x
		x_rot = rot.apply(unit_x)
		x_rot_dash = rot270ZX.apply(x_rot)

		#ax.plot3D([frame['x'], frame['x'] + x_rot[0]],
		#		  [frame['y'], frame['y'] + x_rot[1]],
	#			  [frame['z'], frame['z'] + x_rot[2]], 'r')

		# y
		y_rot = rot.apply(unit_y)
		y_rot_dash = rot270ZX.apply(y_rot)
		ax.plot3D([frame['x'], frame['x'] + y_rot[0]],
				  [frame['y'], frame['y'] + y_rot[1]],
				  [frame['z'], frame['z'] + y_rot[2]], 'b')

		#z
		z_rot = rot.apply(unit_z)
		z_rot_dash = rot270ZX.apply(z_rot)
		#ax.plot3D([frame['x'], frame['x'] + z_rot[0]],
		# 		  [frame['y'], frame['y'] + z_rot[1]],
		#		  [frame['z'], frame['z'] + z_rot[2]], 'g')

	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')

	ax.set_title('SLAM coordinate system')
	ax.set_aspect('equal')


def main():

	data_dir = "/media/tom/HD#39/Year2"
	trajectory_file = os.path.join(data_dir,"A27","Sequences","2368_0","KeyFrameTrajectory_full.txt")
	trajectories = pd.read_csv(trajectory_file, sep=" ", header = None)
	trajectories.columns = ['timestamp','x','y','z','qx','qy','qz','qw']
	plotTrajectories3D(trajectories)

if __name__ == '__main__':
	main()

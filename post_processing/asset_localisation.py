import os, sys
sys.path.insert(0, '/home/tom/Projects/Utils/')
sys.path.insert(0, '/home/tom/Projects/SLAM/coordinate_transformation')
sys.path.insert(0, '/home/tom/Projects/SLAM/asset_load')
from assets import load_assets
from load_assets import load_assets_from_cpp
from load_data import loadTrajectory
from estimate_transformation import *
import numpy as np
from plot_trajectories import *
from scipy.spatial.transform import Rotation
from scrape_elevation_data import elevation_gmaps
from camera import *
import pandas as pd
import math


def getAssetCameraCoordsWorld(asset, GPS):

	camera_t = GPS[['Northing', 'Easting', 'height']].to_numpy()
	rot_y = Rotation.from_euler('zyx', [GPS['Heading'],0,0], degrees=True)
	asset_t = asset[['XCOORD', 'YCOORD', 'HEIGHT']].to_numpy()

	asset_cam = asset_t-camera_t
	asset_cam = [float(a) for a in asset_cam]

	asset_cam_rot = rot_y.apply(asset_cam)
	#print(asset_cam_rot)

	return [asset_cam_rot[0], asset_cam_rot[2], asset_cam_rot[1]]


def assetToCameraFrame(asset_SLAM, camera_SLAM):
	""" Converts the assets from the SLAM coordinate frame to the frame of the camera """
	#camera_t = asset_SLAM[['x','z','y']].to_numpy()

	#camera_R = Rotation.from_quat([camera_SLAM['qx'],camera_SLAM['qy'],camera_SLAM['qz'], camera_SLAM['qw']])
	[yaw, pitch, roll] = quaternion_to_euler(camera_SLAM['qx'],camera_SLAM['qy'],camera_SLAM['qz'], camera_SLAM['qw'])

	#print("yaw, pitch, roll: ", np.rad2deg(yaw), np.rad2deg(pitch), np.rad2deg(roll))
	#print("camera rotation in SLAM: ", camera_R.as_euler('xyz', degrees=True))
	# this is hopefully in the frame of the camera.
	#camera_R_inv = camera_R.inv()
	#asset_CAM = camera_R.apply(asset_SLAM)

	G = yaw
	B = pitch
	A = roll

	R = np.array([[np.cos(G)*np.cos(B), -np.sin(G)*np.cos(A)+np.cos(G)*np.sin(B)*np.sin(A),  np.sin(G)*np.sin(A)+np.cos(G)*np.sin(B)*np.cos(A)],
		  		  [np.sin(G)*np.cos(B),  np.cos(G)*np.cos(A)+np.sin(G)*np.sin(B)*np.sin(A), -np.cos(G)*np.sin(A)+np.sin(G)*np.sin(B)*np.cos(A)],
		 		  [-np.sin(B),        np.cos(B)*np.sin(A),                       np.cos(B)*np.cos(A)]])

	#print(R, camera_t)

	print('--------------')
	print(asset_SLAM)
	print('--------------')

	asset_SLAM = np.array([asset_SLAM[2], asset_SLAM[0], asset_SLAM[1]])

	asset_CAM = np.matmul(R,asset_SLAM)
	# vector between
	camera_to_asset = asset_CAM
	print(asset_SLAM)
	print(asset_CAM)
	print('--------------')

	return camera_to_asset


def projectAsset(asset_CAM):
	""" Projects the asset onto the image from it's coordinate in the slam world."""
	# camera parameters
	fx = 2105.893012
	fy = 2244.311573

	m = 2560
	n = 2048

	u = fx * (asset_CAM[1]/asset_CAM[0]) + (m/2)
	v = fy * (asset_CAM[2]/asset_CAM[0]) + (n/2)

	print(u,v)

	pixels = [u,v]

	return pixels


def getCloseAssets(assets_df, Easting, Northing, max_dist=20):
		assets_dist = np.sqrt((assets_df["Northing"] - Northing)**2 + (assets_df["Easting"] - Easting)**2)
		return assets_df[assets_dist < max_dist]


def main():
	""" The keyframe trajectory file and gps are loaded (in same df) and a 7D estimate_transformation
	between the two coordinate systems is computed. The assets may then be mapped into the SLAM map. As the
	orientation of the camera in the SLAM map is also known, we can project the assets in the SLAM map onto the image """

	camera = SurveyCamera('A27')

	WINDOW_SIZE = 20
	start = 2100
	# we slide of a window of trajectories
	trajectory = loadTrajectory()
	trajectory['Height'] = 2.301 + trajectory['Elevation']
	# load asset file
	data_dir = "/media/tom/HD#39/Year2/A27/Sequences/2368_0/"
	assets_elevation_df = pd.read_csv(os.path.join(data_dir, 'assets_elevation.txt'), sep=' ')
	print(assets_elevation_df)
	#assets_elevation_df['Height'] = assets_elevation_df['Elevation'] + assets_elevation_df['Height']

	#assets_elevation_df['Height'] = 0#assets_elevation_df['Total_Height']
	#assets_A27 = load_assets('A27','SNSF')
	#assets_A27['Height'] = assets_A27['MOUNTING_H']
	#dassets_A27['Height'] = 0
	n_images = len(trajectory)
	# estimation of transform:
	transform = [1,1,1,1,0,0,0]

	for i in range(start,start+WINDOW_SIZE+1000):
		print('-------------------')
		# load frame and the windowtransform_coordinate
		frame = trajectory.iloc[i]
		# camera pose of the frame
		camera_SLAM = frame[['x','y','z','qx','qy','qz','qw']]
		trajectory_window = trajectory.iloc[i:i+WINDOW_SIZE]
		# select 2 coordinate frames from trajectory window to calculate transformation
		trajectory_window_GPS = trajectory_window[['Easting','Height','Northing']]
		#print(trajectory_window_GPS)

		trajectory_window_GPS_translated_df = trajectory_window_GPS - trajectory_window_GPS.iloc[0]
		trajectory_window_GPS_translated = trajectory_window_GPS_translated_df.to_numpy()

		print('------')


		# assets in that window
		assets_df = getCloseAssets(assets_elevation_df, frame['Easting'], frame['Northing'])
		assets_df = assets_df.reset_index(drop=True)
		#assets = assets_df[['Easting','MOUNTING_H','Northing']].to_numpy()
		#plotTrajectoryAndAssets(trajectory_window_SLAM_translated_df, assets_SLAM)
		#plt.show()
		#plt.show()
		in_image = [] # is close asset in the image
		pixels = [] # pixels of those assets
		assets_v = [] # assets coordinates in frame of the vehicle.
		assets_gps = [] # gps tag of asset
		for (ind, asset) in assets_df.iterrows():
			[asset_pixels, asset_v] = camera.projectWorldCoord(asset, frame)
			if asset_pixels[0] > 0 and asset_pixels[0] < 2500 and asset_pixels[1] > 0 and asset_pixels[1] < 2024 and asset_v[0] < 15:
				in_image.append(True)
				pixels.append(asset_pixels)
				assets_v.append(asset_v)
				assets_gps.append([asset['Easting'],asset['Height'],asset['Northing']])
			else:
				in_image.append(False)

		if len(pixels) > 0:
			pixels_gps = np.concatenate(pixels, axis=0).reshape((-1,2))
			assets_v = np.concatenate(assets_v, axis=0).reshape((-1,3))
			assets_gps = np.concatenate(assets_gps, axis=0).reshape((-1,3))

			#print(pixels_gps)
			#print(assets_v)
			#print(assets_gps)
			#plotAssetsOnImage(frame["image_file"], asset_pixels_gps=pixels_gps)
			#plt.show()

		else:
			pixels_gps = []
			assets_gps = None
			assets_SLAM = None

		if assets_gps is not None:
			print("assets gps")
			print(assets_gps)
			print("asset_pixels - from calibration")
			print(pixels_gps)

			e0 = float(trajectory_window_GPS.iloc[0]['Easting'])
			h0 = float(trajectory_window_GPS.iloc[0]['Height'])
			n0 = float(trajectory_window_GPS.iloc[0]['Northing'])

			print('translated assets')
			assets_gps_translated = np.subtract(assets_gps, np.array([e0,h0,n0]))
			print(assets_gps_translated)# plotGPSAndAssets(trajectory_window_GPS_translated_df[['Northing','Total_Height','Easting']], assets)
			# plotting

			# perform translation
			trajectory_window_SLAM = trajectory_window[['x','y','z']]
			trajectory_window_SLAM_translated_df = trajectory_window_SLAM - trajectory_window_SLAM.iloc[0]
			trajectory_window_SLAM_translated_df['qx'] = trajectory_window['qx']
			trajectory_window_SLAM_translated_df['qy'] = trajectory_window['qy']
			trajectory_window_SLAM_translated_df['qz'] = trajectory_window['qz']
			trajectory_window_SLAM_translated_df['qw'] = trajectory_window['qw']


			trajectory_window_SLAM_coords_translated = trajectory_window_SLAM_translated_df[['x','y','z']].to_numpy()
			print('----TRASNFORMATION----')
			print('translated gps:')
			print(trajectory_window_GPS_translated)

			print("translated SLAM:")
			print(trajectory_window_SLAM_coords_translated)
			# estimate transformationn
			transform = estimate_translated_transformation(trajectory_window_GPS_translated,
														   trajectory_window_SLAM_coords_translated, transform)


			slam_from_gps = [transform_translated_coordinate(transform, gps) for gps in trajectory_window_GPS_translated]
			slam_from_gps = np.concatenate(slam_from_gps, axis=0).reshape((-1,3))

			# now transform assets into SLAM world
			assets_SLAM_list = [transform_translated_coordinate(transform, asset) for asset in assets_gps_translated]
			assets_SLAM = np.concatenate(assets_SLAM_list, axis=0).reshape((-1,3))

			#plotTransformedCoords(slam_from_gps, trajectory_window_SLAM_coords_translated)
			#plt.show()
			#sys.exit()
			# getting pixels off road
			road_coords_SLAM = trajectory_window_SLAM_coords_translated
			road_pixels = []
			road_SLAM_CAMs = []
			for road_coord_SLAM in road_coords_SLAM[1:]:
				road_SLAM_CAM = assetToCameraFrame(road_coord_SLAM, camera_SLAM)
				road_SLAM_CAMs.append(road_SLAM_CAM)
				road_pixels_SLAM = projectAsset(road_SLAM_CAM)
				road_pixels.append(road_pixels_SLAM)
			road_pixels = np.concatenate(road_pixels, axis=0).reshape((-1,2))
			road_SLAM_CAMs = np.concatenate(road_SLAM_CAMs, axis=0).reshape((-1,3))
			#plotAssetsOnImage(frame['image_file'], road_pixels)
			#plotSLAMCAMRoad2D(trajectory_window_SLAM_translated_df, frame)
			#plotSLAMCAMRoad(road_SLAM_CAMs, frame)
			plotSLAMRoadPixels(frame['image_file'], road_pixels)
			plt.show()
			sys.exit()
			#plt.figure()

			#sys.exit()
			# loop through assets
			asset_SLAM_CAMs = []
			asset_SLAM_pixels = []
			for (ind, asset_SLAM) in enumerate(assets_SLAM):
				asset_SLAM_CAM = assetToCameraFrame(asset_SLAM, camera_SLAM)
				asset_SLAM_CAMs.append(asset_SLAM_CAM)
				pixels_SLAM = projectAsset(asset_SLAM_CAM)
				asset_SLAM_pixels.append(pixels_SLAM)
				print('----------')
				print('asset_SLAM: ', asset_SLAM)
				print('asset_SLAM_CAM: ', asset_SLAM_CAM)
				print('asset_GPS_CAM:', assets_v[ind,:])
				print('pixels SLAM: ', pixels_SLAM)
				print('pixels GPS:', pixels_gps[ind,:])
				print('----------')


			asset_SLAM_pixels = np.concatenate(asset_SLAM_pixels, axis=0).reshape((-1,2))
			plotAssetsOnImage(frame['image_file'], pixels_gps, asset_SLAM_pixels)
			plt.show()



if __name__ == '__main__':
	main()

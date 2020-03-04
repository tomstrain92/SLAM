import os,sys

import pandas as pd
import requests
import urllib


def data_dir():
	return "/media/tom/HD#39/Year2/A27/Sequences/2368_0/"


def loadTrajectory():
	"""Load trajectory file created by ORB slam and links it with
	 the GPS and original image file"""

	# orb slam trajectory
	#print("Loading trajectories")
	trajectory_file = os.path.join(data_dir(),"KeyFrameTrajectory_full.txt")
	trajectories = pd.read_csv(trajectory_file, sep=" ", header = None)
	trajectories.columns = ['timestamp','x','y','z','qx','qy','qz','qw']
	#print(trajectories.head())

	# gps
	#print("Loading gps")
	gps_file = os.path.join(data_dir(), "image_link_gps_elevation.txt")
	gps = pd.read_csv(gps_file, sep=" ", header = None)
	gps.columns = ['index','timestamp','Easting','Northing','HEADING','Lon','Lat','Elevation']

	gps['XCOORD'] = gps['Easting']
	gps['YCOORD'] = gps['Northing']
	gps['height'] = 2.3

	# images
	images_file = os.path.join(data_dir(), "rgb.txt")
	images = pd.read_csv(images_file, sep=" ", header = None)
	images.columns = ['timestamp','image_file']

	#print(images)

	joined_trajectory = trajectories.set_index('timestamp').join(gps.set_index('timestamp')).join(images.set_index('timestamp'))

	#print(joined_trajectory)

	return joined_trajectory

#trajectories = loadKeyFrameTrajectoriesWithGPS()

import requests
import urllib
import pandas as pd
import json
from convertbng.util import convert_lonlat
from progress.bar import Bar
import os, sys


def add_lat_lon(gps):
	# getting height elevation
	[lon, lat] = convert_lonlat(gps['Easting'].to_numpy(),gps['Northing'].to_numpy())
	gps['lon'] = lon
	gps['lat'] = lat

	return gps


def scape_evelation(gps_df):

	bar = Bar('Scraping elevations', max=len(gps_df))
	elevations = []
	for (ind, gps) in gps_df.iterrows():
		evelation = elevation_gmaps(float(gps['lat']),float(gps['lon']))
		elevations.append(evelation)
		bar.next()
	bar.finish()

	return elevations


def elevation_gmaps(lat, lon):

	apikey = "AIzaSyBOMc1dHgyUHXj1ZrkazxgGh03h_NjjkRs"
	url = "https://maps.googleapis.com/maps/api/elevation/json"
	request = requests.get(url+"?locations="+str(lat)+","+str(lon)+"&key="+apikey)
	try:
	    results = json.loads(request.text).get('results')
	    if 0 < len(results):
	        elevation = results[0].get('elevation')
	        #resolution = results[0].get('resolution') # for RESOLUTION
	        # ELEVATION
	        return elevation
	    else:
	        print('HTTP GET Request failed.')
	except ValueError as e:
	    print('JSON decode failed: '+str(request) + str(e))


def elevation_nationalmap(lat, lon):
	"""Query service using lat, lon. add the elevation values as a new column."""

	# define rest query params
	params = {
	'output': 'json',
	'x': lon,
	'y': lat,
	'units': 'Meters'
	}

	# USGS Elevation Point Query Service
	url = r'https://nationalmap.gov/epqs/pqs.php?'

	# format query string and return query value
	result = requests.get((url + urllib.parse.urlencode(params)))
	elevation = result.json()['USGS_Elevation_Point_Query_Service']['Elevation_Query']['Elevation']

	return elevation


def main():

	data_dir = "/media/tom/HD#39/Year2/A27/Sequences/2368_0/"
	image_link_gps_file = os.path.join(data_dir, "image_link_gps.txt")
	gps = pd.read_csv(image_link_gps_file, sep=" ", header=None)
	gps.columns = [['timestamp','Easting','Northing','Heading']]
	# add lat lon for gmaps api.
	gps = add_lat_lon(gps)
	# call gmaps api..
	evelations = scape_evelation(gps)
	gps['evelation'] = evelations
	# save
	gps.to_csv(os.path.join(data_dir, "image_link_gps_elevation.txt"), sep=" ", header=None)

if __name__ == '__main__':
	main()

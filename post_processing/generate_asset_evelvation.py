from scrape_elevation_data import *
import sys
sys.path.insert(0, '/home/tom/Projects/Utils/')
from assets import load_assets
from progress.bar import Bar
import numpy as np
import pandas as pd

def main():

	asset_types = ['SNSF','MKRF']
	asset_df = pd.DataFrame(columns=['Easting', 'Northing'])


	for asset_type in asset_types:
		assets = load_assets("A27", asset_type)

		assets['Easting'] = assets['XCOORD']
		assets['Northing'] = assets['YCOORD']

		# adding lat Lon
		bar = Bar("Scraping elevation data for {}...".format(asset_type), max=len(assets))
		assets = add_lat_lon(assets)
		elevations = []
		for (ind, asset) in assets.iterrows():
			# get elevation and add to to height.

			elevation = elevation_gmaps(float(asset['lat']),float(asset['lon']))
			asset_df = asset_df.append({

				'Easting' : asset['Easting'],
				'Northing' : asset['Northing'],
				'Elevation' : elevation,
				'Height' : asset['HEIGHT'],
				'Type' : asset_type

			}, ignore_index=True)

			bar.next()
		bar.finish()

	# save
	data_dir = "/media/tom/HD#39/Year2/A27/Sequences/2368_0/"
	asset_df.to_csv(os.path.join(data_dir, "assets_elevation.txt"), sep=' ', header=True, index=False)

if __name__ == '__main__':
	main()

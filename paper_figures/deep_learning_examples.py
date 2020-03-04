import os
import sys
sys.path.insert(0, "/home/tom/Projects/SLAM/vehicle_detection/")
from yoloDetectorGPU import *
import cv2

def main():
	# load images_file
	image_dir = "images"
	out_dir = "vehicles"
	images = os.listdir(image_dir)

	# load detector
	yoloDetector = load_detector()

	for image in images:
		img = cv2.imread(os.path.join(image_dir, image))
		[boxes, annotated_img] = run_inference(yoloDetector, img)
		cv2.imwrite(os.path.join(out_dir, image), annotated_img)

if __name__ == '__main__':
	main()

#detector = load_detector()

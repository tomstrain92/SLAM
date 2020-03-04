from io import BytesIO
import numpy as np
from timeit import default_timer as timer
from pydarknet import Detector, Image
import cv2
from nms.nms import boxes


def load_detector():
	# constructor, load detector
	code_dir = "/home/tom/Projects/SLAM/vehicle_detection/"
	net = Detector(bytes(code_dir + "cfg/yolov3.cfg", encoding="utf-8"),
			bytes(code_dir + "cfg/yolov3.weights", encoding="utf-8"), 0,
            bytes(code_dir + "cfg/coco.data", encoding="utf-8"))
	return net


def run_inference(net, img):
	# run inference of an image - eventually I will have to change
	#print("running inference")
	show=False
	vehicle_classes = ['car', 'vehicle,', 'truck', 'bus', 'bike', 'motorbike']
	# not sure what this does
	img2 = Image(img)
	# running inference:
	start = timer()
	results = net.detect(img2)
	end = timer()

	results_labels = [x[0].decode("utf-8") for x in results]

	# finding vehicles
	vehicle_boxes =  np.empty((0,4), int)
	buffer = 0

	# running non max supression
	scores = [score for cat, score, bound in results]
	bounds = [bound for cat, score, bound in results]
	indices = boxes(bounds, scores)
	results = [results[ind] for ind in indices]

	# box buffer
	buffer = 10
	for cat, score, bounds in results:
		x, y, w, h = bounds # x,y are centre.
		if cat.decode("utf-8") in vehicle_classes:
			vehicle_boxes = np.append(vehicle_boxes,
								np.array([[int(x - w / 2) - buffer,
										   int(y - h / 2) - buffer,
								 		   int(w) + buffer, int(h) + buffer]]))
		# plotting results if you so wish
		if show:
			cv2.rectangle(img, (int(x - w / 2), int(y - h / 2)),
				(int(x + w / 2), int(y + h / 2)), (0, 0, 255), 5)

		#cv2.putText(img, str(cat.decode("utf-8")),(int(x),int(y)),
			#cv2.FONT_HERSHEY_COMPLEX,1,(255,255,0))
	if show:
		cv2.imshow("output", img)
		cv2.waitKey(0)

	#num_vehicles = int(len(vehicle_boxes)/4)
	#print("detection took {:f} seconds, {:d} vehicles found".format(end-start,num_vehicles))
	return vehicle_boxes


if __name__ == '__main__':

	detector = load_detector()
	img = cv2.imread("../cars.jpg")
	[vehicle_boxes, img_vehicles] = run_inference(detector,img)

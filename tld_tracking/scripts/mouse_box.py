#!/usr/bin/env python
import cv2
import argparse

ref_points = []
bounding_boxes = []

def mouse_draw(event, x, y, flags, param):
	global ref_points
	print  "Hello"
	if event == cv2.EVENT_LBUTTONDOWN:
		print event
		ref_points = [(x,y)]
		print ref_points
		print "The key is pressed, value %d, %d" %(ref_points[0][0], ref_points[0][1])

	elif event == cv2.EVENT_LBUTTONUP:
		ref_points.append((x,y))
		print ref_points
		cv2.rectangle(image, ref_points[0], ref_points[1], (0,255,0),2)
		bounding_boxes.append([ref_points[0], ref_points[1]])
		print bounding_boxes
		print "The key is released, value %d, %d" %(ref_points[1][0], ref_points[1][1])


argument_object = argparse.ArgumentParser()
argument_object.add_argument("-i", "--image", required=True, help="Path to the image")
arguments = vars(argument_object.parse_args())


image = cv2.imread(arguments["image"])
clone_image = image.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image", mouse_draw, image)

while True:
	cv2.imshow("image", image)
	key = cv2.waitKey(1) & 0xFF

	if key == ord("r"):
		image = clone_image.copy()

	if key == ord("c"):
		break

cv2.destroyAllWindows()



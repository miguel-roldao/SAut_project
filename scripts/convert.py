import cv2
import numpy as np
import matplotlib.pyplot as plt


def pixel_to_coordinates(px:int, py:int, xpix:int, ypix:int, xmin:float, xmax:float) -> (float,float):
	"""
	px: position in pixels of the x
	py: position in pixels of the y
	xpix: number of pixels in x direction
	ypix: number of pixels in y direction
	xmin and xmax: range of the plot (real coordinates)

	return:
	x real coordinate
	y real coordinate

	------------>						
	|			py 						Λ
	|									| y
	|					---->			|
	|									|
	|px 								|
	v 									|		  x
										----------->
	"""

	xdelta = xmax - xmin

	x = xmin + xdelta*py/ypix
	y = xmin + xdelta*(xpix-px)/xpix

	return x,y



if __name__== '__main__':

	x=[-20,-20,-20,0,0,0,20,20,20]
	y=[-20,0,20,-20,0,20,-20,0,20]
	
	plt.scatter(x,y)
	#plt.scatter(2,2)
	plt.axis("off")
	plt.xlim(-20,20)
	plt.ylim(-20,20)
	plt.axis("scaled")

	plt.savefig('pixel_test.png', bbox_inches='tight', pad_inches=-0.15)

	img = cv2.imread('pixel_test.png')

	print("rows:" + str(len(img)))
	print("cols:" + str(len(img[0])))

	px=7
	py=300

	x0, y0 = pixel_to_coordinates(px,py,len(img[0]),len(img),-20,20)
	print((x0,y0))
	for i in range(-5,5):
		for j in range(-5,5):
			img[px+i,py+j]=[0,0,255]

	cv2.imwrite("result.png",img)

	plt.scatter(x0, y0)
	plt.savefig('pixel_test.png', bbox_inches='tight', pad_inches=-0.15)
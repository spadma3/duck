import kmeans
import plot_rgb
import cv2
import random
import numpy as np

class AntiInstagram(object):
	def __init__(self):
		self.num_colors = 3
		self.ideal_colors = [[0, 0, 0], [0, 255, 255], [255, 255, 255]]
		self.transformation = [0,0,0]
		self.scale = [0,0,0]
		self.shift = [0,0,0]
		self.health = 0

	def applyTransform(self,image):
		corrected_image = kmeans.scaleandshift(image,self.scale,self.shift)
		return corrected_image

	def calculateTransform(self,image):

		trained,counter = kmeans.runKMeans(image)
		mapping = kmeans.identifyColors(trained, kmeans.CENTERS)
		r,g,b = kmeans.getparameters(mapping, trained, kmeans.CENTERS)
		if r[0][0][0] == 0.0: return

		self.scale = [r[0][0][0],g[0][0][0],b[0][0][0]]
		self.shift = [r[1][0], g[1][0],b[1][0]]

		return trained

	def calculateHealth(self):
		'''one way is to keep one img every 1 sec and compare the diff http://stackoverflow.com/questions/189943/how-can-i-quantify-difference-between-two-images'''

		

		return self.health
		

	def calibrateLighting(self, cal_img):
		sample = self.getSample(cal_img)

		equalParam = self.histEqual(sample)

		kMeansParam = self.kMeansParam(sample)

		return self.calibration_parameters

	def sampleAndSlice(self, img, numSamples=100):
		'''code to sample from entire img a part that contains lanes and floor'''
		img = img[-100:,:,:]
		# cv2.imshow('image',img)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		iList = [random.randint(0,len(img)-1) for i in range(numSamples)]
		jList = [random.randint(0,len(img[0])-1) for i in range(numSamples)]
		# print iList,jList
		sampled = np.asarray([img[i,j,:] for i in iList for j in jList])
		# print sampled
		return sampled

	def histEqual(self, img):
		'''https://en.wikipedia.org/wiki/Normalization_(image_processing)'''

		return equalParam

	def kMeans(self, img):

		return kMeansParam

	def plotRGB(self,img,centers=None):
		plot_rgb.all(img,60,c=centers)

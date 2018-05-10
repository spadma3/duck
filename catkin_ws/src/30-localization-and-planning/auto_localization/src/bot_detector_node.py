#!/usr/bin/env python
import time
import cv2

from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped

BG_SAMPLE = 150
BG_THRESHOLD = 1000000

class BotDetectorNode(object):
	"""docstring for BotDetectorNode"""
	def __init__(self):
		self.node_name = "BotDetectorNode"

		#Constructor of bot detector
		self.bridge = CvBridge()

		self.active  = True


		#Initial Background and counter
		self.background = []
		self.count = 0

		#Define doing getBackground or Doing Botdetect
		if rospy.get_param('~bot', False):
			self.func = 'getDuckiebot'
		else:
			self.func = 'getBackground'

		if self.func == 'getDuckiebot':
			self.background = cv2.imread('/home/erickiemvp/duckietown/background.png', cv2.IMREAD_COLOR)
			self.background = self.background.astype(np.float64)

		#Publisher
		self.pub_result = rospy.Publisher("~bot_detection", BoolStamped, queue_size=1)

		#Subscriber
		self.sub_image = rospy.Subscriber("image", Image, self.cbImage, queue_size=1)
		self.subswitch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)

		#Record the first number of sequence
		self.first_seq = -1

		rospy.loginfo("[%s] Initialized " %(self.node_name))

		#rospy.Timer(rospy.Duration.from_sec(2.0), self.updateParams)

	'''
	def updateParams(self):
		old_verbose = self.verbose
		self.verbose = rospy.get_param('~verbose', True)

		if self.verbose != old_verbose:
			self.loginfo('Verbose is now %r' % self.verbose)

		self.image_size = rospy.get_param('~img_size')
		self.top_cutoff = rospy.get_param('~top_cutoff')
	'''

	def cbSwitch(self, switch_msg):
		self.active = switch_msg.data

	def cbImage(self, image_msg):

		if not self.active:
			return
		if self.func == 'getBackground':
			self.getBackground(image_msg)
		elif self.func == 'getDuckiebot':
			self.getDuckiebot(image_msg)

	def loginfo(self, s):
		rospy.loginfo('[%s] %s' % (self.node_name, s))

	def getDuckiebot(self, image_msg):

		'''
		# Decode from compressed image with OpenCV
		try:
			image_cv = image_cv_from_jpg(image_msg.data)
		except ValueError as e:
			self.loginfo('Could not load image: %s' % e)
			return
		'''
		'''
		# Record the first number of seq
		image_cv = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
		if self.first_seq == -1:
			self.first_seq = image_msg.header.seq

		n = image_msg.header.seq - self.first_seq
		rospy.loginfo('Its %d image' % (n))
		if n <= BG_SAMPLE:
			#Compute background with the first 1000 images
			if n == 0:
				self.background = image_cv
				self.bg_b, self.bg_g, self.bg_r = cv2.split(self.background)
			else:
				self.bg_b, self.bg_g, self.bg_r = self.getMidBackground(self.bg_b, self.bg_g, self.bg_r, image_cv)

			return

		elif n == (BG_SAMPLE+1):
			self.background = cv2.merge(self.bg_b[:, :, BG_SAMPLE/2], self.bg_g[:, :, BG_SAMPLE/2], self.bg_r[:, :, BG_SAMPLE/2])
		'''

		image_cv = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
		image_cv = image_cv.astype(np.float64)

		bg_sub = np.subtract(image_cv, self.background)
		bg_abs = np.absolute(bg_sub)
		bg_abs[bg_abs<50] = 0
		self.img_sum = np.sum(bg_abs)

		#self.loginfo('Background r sum: %d' % (self.bg_sum))
		#self.loginfo('Image r sum: %d' % (self.img_sum))
		self.loginfo('Subtraction absolute sum = %d' %(self.img_sum))

		result = BoolStamped()
		result.header = image_msg.header
		if self.img_sum > BG_THRESHOLD:
			self.loginfo('True')
			result.data = True
		else:
			self.loginfo('False')
			result.data = False
			
		self.pub_result.publish(result)

	def getBackground(self, image_msg):
		image_cv = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
		if self.first_seq == -1:
			self.first_seq = image_msg.header.seq

		n = image_msg.header.seq - self.first_seq
		rospy.loginfo('Its %d image' % (n))
		rospy.loginfo('Counts %d' % (self.count))

		if self.count <= BG_SAMPLE:
			#Compute background with the first 1000 images
			if self.count == 0:
				self.bg_b, self.bg_g, self.bg_r = cv2.split(image_cv)
				self.count+=1
				self.bg_b = np.expand_dims(self.bg_b, axis=2)
				self.bg_g = np.expand_dims(self.bg_g, axis=2)
				self.bg_r = np.expand_dims(self.bg_r, axis=2)
			else:
				self.bg_b, self.bg_g, self.bg_r = self.appendImage(self.bg_b, self.bg_g, self.bg_r, image_cv)
				self.count+=1
			return
		else:
			#Save background into file, do it once.
			self.loginfo('Finish append, Start sorting')
			self.active = False
			self.MidBackground = self.getMidBackground(self.bg_b, self.bg_g, self.bg_r)
			saveResult = cv2.imwrite('/home/erickiemvp/duckietown/background.png', self.MidBackground)
			print "Save Result = ", saveResult
			self.loginfo('Get Background Done! You can shutdown now.')
			return

	def appendImage(self, bg_b, bg_g, bg_r, img):
		
		b, g, r = cv2.split(img)

		#Append at beginning
		#ap_index = 0
		#Append at the end
		ap_index = bg_b.shape[2]

		bg_b = np.insert(bg_b, ap_index, b, axis=2)
		bg_g = np.insert(bg_g, ap_index, g, axis=2)
		bg_r = np.insert(bg_r, ap_index, r, axis=2)

		return bg_b, bg_g, bg_r

	def getMidBackground(self, bg_b, bg_g, bg_r):

		start = rospy.get_time()

		bg_b = np.sort(bg_b, kind='mergesort')
		bg_g = np.sort(bg_g, kind='mergesort')
		bg_r = np.sort(bg_r, kind='mergesort')

		bg_b = bg_b[:, :, BG_SAMPLE/2]
		bg_g = bg_g[:, :, BG_SAMPLE/2]
		bg_r = bg_r[:, :, BG_SAMPLE/2]

		result_img = cv2.merge((bg_b, bg_g, bg_r))

		end = rospy.get_time()
		print "Time of running this image = ", (end - start)

		return result_img

	def on_Shutdown(self):
		self.loginfo("Shutdown.")

'''
class Stats(object):
	"""docstring for Stats"""
	def __init__(self):
		self.nresets = 0
		self.reset()

	def reset(self):
		self.nresets += 1
		self.t0 = time.time()
		self.nreceived = 0
		self.nskipped = 0
		self.nprocessed = 0
	
	def received(self):
		if self.nreceived==0 and self.nresets==1:
			rospy.loginfo('bot_detector_node received first image')

	def skipped(self):
		self.nskipped += 1

	def processed(self):
		if nreceived==0 and self.nresets==1:
			rospy.loginfo('bot_detector_node processing first image')

		self.nprocessed += 1

	def info(self):
		delta = time.time() - self.t0

		if self.nreceived:
			skipped_perc = (100 * self.nskipped / self.nreceived)
		else:
			skipped_perc = 0

		def fps(x):
			return '.1f fps' % (x / delta)

		m = 'In the last %.1f s: received %d (%s) processed %d (%s) skipped %d (%s) (%1.f%%)' % (delta, self.nreceived, fps(self.nreceived), self.nprocessed, fps(self.processed), self.nskipped, fps(self.nskipped), skipped_perc)
		return m 
'''
		


if __name__ == '__main__':
	rospy.init_node('bot_detector', anonymous=False)
	bot_detector_node=BotDetectorNode()
	rospy.on_shutdown(bot_detector_node.on_Shutdown)
	rospy.spin()

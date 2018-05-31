#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32

# DVisual Odometry node
# Author: Gianmarco Bernasconi/Julien Kindle
# Inputs: ~Image/Image - Input raw image
# Outputs: ~pose/Int32 - The output pose

class PinholeCamera:
	def __init__(self, width, height, fx, fy, cx, cy,
				k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
		self.width = width
		self.height = height
		self.fx = fx
		self.fy = fy
		self.cx = cx
		self.cy = cy
		self.distortion = (abs(k1) > 0.0000001)
		self.d = [k1, k2, p1, p2, k3]


class VisualOdometry:
	def __init__(self, cam, velocity):
        self.node_name = 'visual_odometry_node'


		self.frame_stage = 0
		self.cam = cam
		self.new_frame = None
		self.last_frame = None
		self.cur_R = None
		self.cur_t = None
		self.px_ref = None
		self.px_cur = None
		self.focal = cam.fx
		self.pp = (cam.cx, cam.cy)
		self.trueX, self.trueY, self.trueZ = 0, 0, 0
		self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
		self.velocity = velocity

        # Setup the publisher and subscriber
        self.sub_img = rospy.Subscriber("~img", Image, self.Image)

        rospy.loginfo("[%s] has started", self.node_name)

	def getAbsoluteScale(self, frame_id):
        #TODO calculate driven distance between frames and return it here
		return 1

	def processFirstFrame(self):
		self.px_ref = self.detector.detect(self.new_frame)
		self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
		self.frame_stage = STAGE_SECOND_FRAME

	def processSecondFrame(self):
		self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		_, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
		self.frame_stage = STAGE_DEFAULT_FRAME
		self.px_ref = self.px_cur

	def processFrame(self, frame_id):
                if not self.px_ref.any(): return
		self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		if E is None: return
                _, R, t, mask = cv2.recoverPose(E.copy(), self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
		absolute_scale = self.getAbsoluteScale(frame_id)
		if(absolute_scale > 0.1): # TODO: if we change scaling, this needs to be changes as well I think
			self.cur_t = self.cur_t + absolute_scale*self.cur_R.dot(t)
			self.cur_R = R.dot(self.cur_R)
		if(self.px_ref.shape[0] < kMinNumFeature):
			self.px_cur = self.detector.detect(self.new_frame)
			self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
		self.px_ref = self.px_cur

	def update(self, img, frame_id):
		assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
		self.new_frame = img
		if(self.frame_stage == STAGE_DEFAULT_FRAME):
			self.processFrame(frame_id)
		elif(self.frame_stage == STAGE_SECOND_FRAME):
			self.processSecondFrame()
		elif(self.frame_stage == STAGE_FIRST_FRAME):
			self.processFirstFrame()
		self.last_frame = self.new_frame

    def featureTracking(image_ref, image_cur, px_ref):
    	kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]

    	st = st.reshape(st.shape[0])
    	kp1 = px_ref[st == 1]
    	kp2 = kp2[st == 1]

    	return kp1, kp2

    def ImageCallback(self, msg_quacks):
        msg_duckiecall = String()
        msg_duckiecall.data = self.quacker.get_quack_string(msg_quacks.data)
        self.pub_duckiecall.publish(msg_duckiecall)





if __name__ == '__main__':
    rospy.init_node('visual_odometry_node', anonymous=False)

    rospy.spin()

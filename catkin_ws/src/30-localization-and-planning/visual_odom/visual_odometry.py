import numpy as np
import cv2
import time

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2

#TODO: It was 1500 before, I think 5 is fine 1500 gives just shit
kMinNumFeature = 50

# No idea what this is
lk_params = dict(winSize  = (21, 21),
                #maxLevel = 3,
                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def featureTracking(image_ref, image_cur, px_ref):
    kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]

    st = st.reshape(st.shape[0])
    kp1 = px_ref[st == 1]
    kp2 = kp2[st == 1]

    return kp1, kp2


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
        self.time_now=0

    def getAbsoluteScale(self):
        #TODO calculate driven distance between frames and return it here
        self.time_now=time.time()
        self.elapsed_time = self.time_now - self.time_last
        print self.elapsed_time
        self.time_last = self.time_now
        return self.velocity * self.elapsed_time


    def processFirstFrame(self):
        self.px_ref = self.detector.detect(self.new_frame)
        self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
        self.frame_stage = STAGE_SECOND_FRAME
        self.time_last = time.time()

    def processSecondFrame(self):
        self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
        E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
        #self.cur_t=self.cur_t*10**(-9)
        #print self.cur_t
        self.frame_stage = STAGE_DEFAULT_FRAME
        self.px_ref = self.px_cur
        self.time_last = time.time()

    def processFrame(self, frame_id):
        if not self.px_ref.any():
            return
        self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
        if not self.px_cur.any():
            return
        E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return
        _, R, t, mask = cv2.recoverPose(E.copy(), self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
        absolute_scale = self.getAbsoluteScale()
        if(absolute_scale > 0.01): # TODO: if we change scaling, this needs to be changed as well, I think
            #print self.cur_t
            self.cur_t = self.cur_t + absolute_scale*self.cur_R.dot(t)
            self.cur_R = R.dot(self.cur_R)
        if(self.px_ref.shape[0] < kMinNumFeature):
            self.px_cur = self.detector.detect(self.new_frame)
            self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
        self.px_ref = self.px_cur

    def update(self, img):
        assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
        self.new_frame = img
        if(self.frame_stage == STAGE_DEFAULT_FRAME):
            self.processFrame()
        elif(self.frame_stage == STAGE_SECOND_FRAME):
            self.processSecondFrame()
        elif(self.frame_stage == STAGE_FIRST_FRAME):
            self.processFirstFrame()
        self.last_frame = self.new_frame

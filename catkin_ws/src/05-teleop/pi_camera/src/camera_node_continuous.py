#!/usr/bin/env python
import rospy
import cv2
import io
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from picamera import PiCamera
from picamera.array import PiRGBArray

def colorFilter(img, color, thresh=None):
    # threshold colors in HSV space
    '''TODO: use the previous kmeans estimate +- some range to be the threshold colors'''
    if thresh is not None:
        thresh = np.resize(np.uint8(thresh), (thresh.shape[0], 1, 3))
        thresh = cv2.cvtColor(thresh, cv2.COLOR_BGR2HSV)
        t1, t2 = thresh[0:2, ...]
    if color == 'white':
        if thresh is None:
            t1, t2 = hsv_white1, hsv_white2
        bw = cv2.inRange(img, t1, t2)
    elif color == 'yellow':
        if thresh is None:
            t1, t2 = hsv_yellow1, hsv_yellow2
        bw = cv2.inRange(img, t1, t2)
    elif color == 'red':
        if thresh is None:
            t1, t2, t3, t4 = hsv_red1, hsv_red2, hsv_red3, hsv_red4
        else:
            t3, t4 = thresh[2:4, ...]
        bw1 = cv2.inRange(img, t1, t2)
        bw2 = cv2.inRange(img, t3, t4)
        bw = cv2.bitwise_or(bw1, bw2)
    else:
        raise Exception('Error: Undefined color strings...')

    # binary dilation
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    bw = cv2.dilate(bw, kernel)
    return bw


def processGeom(img, viz=False):
    # first narrow scope to surface of lane to avoid extraneous info
    surf = identifyLaneSurface(img, use_hsv=False, grad_thresh=30)
    img = np.expand_dims(surf, -1) * img
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    colors = ['white', 'yellow', 'red']
    masks = {}
    for color in colors:
        # filter colors to get binary mask
        bw = colorFilter(hsv, color, thresh=None)

        # find contours based on binary mask, filter out small noise
        _, contours, _ = cv2.findContours(bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [contour for contour in contours if contour.shape[0] >= 30]

        # create the masks, store only when nonempty
        mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        cv2.drawContours(mask, contours, -1, 1, -1)
        if len(np.unique(mask)) > 1:
            masks[color] = mask

    if viz:
        tot = np.zeros_like(img)
        for col in masks:
            mimg = np.expand_dims(masks[col], -1) * img
            tot += mimg

    return tot


# this has problems with specular reflections
def identifyLaneSurface(img, use_hsv=False, visualize=False, grad_thresh=30):
    h, w = img.shape[:2]
    img_for_grad = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) if use_hsv else img
    dx = cv2.Sobel(img_for_grad, cv2.CV_32F, 1, 0)
    dy = cv2.Sobel(img_for_grad, cv2.CV_32F, 0, 1)
    grad = (np.sqrt(np.mean(dx ** 2 + dy ** 2, 2)) > grad_thresh).astype(np.uint8)
    mask = np.zeros((h + 2, w + 2), np.uint8)
    y = int(h / 2.0 * 1)
    for x in range(w):
        if not (grad[y, x] or mask[y + 1, x + 1]):
            cv2.floodFill(grad, mask, (x, y), 0,
                          flags=cv2.FLOODFILL_MASK_ONLY)
    mask[y + 1:, :] = 1
    mask[0, :] = 0
    mask = mask[:, 1:-1]
    mask_ = np.zeros((h + 4, w + 2), np.uint8)
    cv2.floodFill(mask, mask_, (0, 0), 0, flags=cv2.FLOODFILL_MASK_ONLY)
    mask = 1 - mask_[2:-2, 1:-1]

    # added to get rid of wall artifacts post filling
    m = np.zeros_like(mask)
    m[mask.shape[0] / 3:, :] = 1
    mask *= m
    return mask

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))
        # TODO: load parameters

        self.framerate = self.setupParam("~framerate",60.0)
        self.res_w = self.setupParam("~res_w",320)
        self.res_h = self.setupParam("~res_h",200)

        # self.img_low_framerate = self.setupParam("~img_low_framerate",30.0)
        # self.img_high_framerate = self.setupParam("~img_high_framerate",5.0)
        # self.img_low_res_w = self.setupParam("~img_low_res_w",320)
        # self.img_low_res_h = self.setupParam("~img_low_res_h",200)
        # self.img_high_res_w = self.setupParam("~img_high_res_w",640)
        # self.img_high_res_h = self.setupParam("~img_high_res_h",400)
        # self.uncompress = self.setupParam("~uncompress",False)

        # TODO: load camera info yaml file and publish CameraInfo
        self.pub_img= rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)

        # if self.uncompress:
        #     self.pub_img_low = rospy.Publisher("~img_low/raw",Image,queue_size=1)
        #     self.pub_img_high= rospy.Publisher("~img_high/raw",Image,queue_size=1)
        # else:
        #     self.pub_img_low = rospy.Publisher("~img_low/compressed",CompressedImage,queue_size=1)
        #     self.pub_img_high= rospy.Publisher("~img_high/compressed",CompressedImage,queue_size=1)

        self.has_published = False
        self.bridge = CvBridge()

        # Setup PiCamera
        self.stream = io.BytesIO()
        self.bridge = CvBridge()
        self.camera = PiCamera()
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w,self.res_h)

        # TODO setup other parameters of the camera such as exposure and white balance etc

        # Setup timer
        self.camera_capture = self.camera.capture_continuous(self.stream,'jpeg',use_video_port=True)
        self.timer_img_low = rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate),self.cbTimer)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTimer(self,event):
        if not rospy.is_shutdown():
            self.camera_capture.next()
            self.grabAndPublish(self.stream,self.pub_img)
            # Maybe for every 5 img_low, change the setting of the camera and capture a higher res img and publish.

    def grabAndPublish(self,stream,publisher):
        # Grab image from stream
        stream.seek(0)
        img_data = stream.getvalue()
        print (img_data.type)
        img_data = processGeom(img_data,viz=True)
        if self.uncompress:
            # Publish raw image
            data = np.fromstring(img_data, dtype=np.uint8)
            image = cv2.imdecode(data, 1)
            image_msg = self.bridge.cv2_to_imgmsg(image)
        else:
            # Publish compressed image only
            image_msg = CompressedImage()
            image_msg.data = img_data
            image_msg.format = "jpeg"

        image_msg.header.stamp = rospy.Time.now()
        # Publish
        publisher.publish(image_msg)
        # Clear stream
        stream.seek(0)
        stream.truncate()

        if not self.has_published:
            rospy.loginfo("[%s] Published the first image." %(self.node_name))
            self.has_published = True

    def onShutdown(self):
        self.camera.close()
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('camera_node',anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    rospy.spin()

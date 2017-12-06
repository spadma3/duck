#!/usr/bin/env python
import rospkg
import rospy
import yaml
import thread
import io

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from picamera import PiCamera
from picamera.array import PiRGBArray
from duckietown_utils import get_duckiefleet_root
from duckietown_msgs.msg import BoolStamped

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

        self.framerate_high = self.setupParam("~framerate_high",30.0)
        self.framerate_low = self.setupParam("~framerate_low",15.0)
        self.res_w = self.setupParam("~res_w",640)
        self.res_h = self.setupParam("~res_h",480)

        self.image_msg = CompressedImage()

        # Setup PiCamera

        self.camera = PiCamera()
        self.framerate = self.framerate_high # default to high
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w,self.res_h)


        # For intrinsic calibration
        self.cali_file_folder = get_duckiefleet_root() + "/calibrations/camera_intrinsic/"
    
        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"

        self.has_published = False
        self.pub_img= rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)
        self.sub_switch_high = rospy.Subscriber("~framerate_high_switch", BoolStamped, self.cbSwitchHigh, queue_size=1)

        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info",SetCameraInfo,self.cbSrvSetCameraInfo)

        self.stream = io.BytesIO()
 
        #self.camera.exposure_mode = 'off'
        # self.camera.awb_mode = 'off'

        self.is_shutdown = False
        self.update_framerate = False
        # Setup timer
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbSwitchHigh(self, switch_msg):
        print switch_msg
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True
 
    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            gen =  self.grabAndPublish(self.stream,self.pub_img)
            try:
                self.camera.capture_sequence(gen,'jpeg',use_video_port=True,splitter_port=0)
            except StopIteration:
                pass
            print "updating framerate"
            self.camera.framerate = self.framerate
            self.update_framerate=False

        self.camera.close()
        rospy.loginfo("[%s] Capture Ended." %(self.node_name))

    def grabAndPublish(self,stream,publisher):
        while not self.update_framerate and not self.is_shutdown and not rospy.is_shutdown(): 
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()
            print (stream_data.type)
            stream_data = processGeom(stream_data,viz=True)
            # Generate compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = stream_data

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            publisher.publish(image_msg)
                        
            # Clear stream
            stream.seek(0)
            stream.truncate()
            
            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." %(self.node_name))
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." %(self.node_name))
        self.is_shutdown=True
        rospy.loginfo("[%s] Shutdown." %(self.node_name))


    def cbSrvSetCameraInfo(self,req):
        # TODO: save req.camera_info to yaml file
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info,filename)
        response.status_message = "Write to %s" %filename #TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" %(filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
        'image_height': camera_info_msg.height,
        'camera_name': rospy.get_name().strip("/"), #TODO check this
        'distortion_model': camera_info_msg.distortion_model,
        'distortion_coefficients': {'data': camera_info_msg.D, 'rows':1, 'cols':5},
        'camera_matrix': {'data': camera_info_msg.K, 'rows':3, 'cols':3},
        'rectification_matrix': {'data': camera_info_msg.R, 'rows':3, 'cols':3},
        'projection_matrix': {'data': camera_info_msg.P,'rows':3, 'cols':4}}
        
        rospy.loginfo("[saveCameraInfo] calib %s" %(calib))

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False

if __name__ == '__main__': 
    rospy.init_node('camera',anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    thread.start_new_thread(camera_node.startCapturing, ())
    rospy.spin()

from cv_bridge import CvBridge  # @UnresolvedImport

__all__ = ['d8n_image_msg_from_cv_image']

class ImageConversions():
    # We only instantiate the bridge once
    bridge = None
    
def d8n_image_msg_from_cv_image(cv_image, image_format, same_timestamp_as = None):
    """ 
        Makes an Image message from a CV image. 
    
        if same_timestamp_as is not None, we copy the timestamp
        from that image.
        
        image_format: 'bgr8' or 'mono' or similar
    """
    if ImageConversions.bridge is None:
        ImageConversions.bridge = CvBridge()
    image_msg_out = ImageConversions.bridge.cv2_to_imgmsg(cv_image, image_format)
    if same_timestamp_as is not None:
        image_msg_out.header.stamp = same_timestamp_as.header.stamp
    return image_msg_out
    

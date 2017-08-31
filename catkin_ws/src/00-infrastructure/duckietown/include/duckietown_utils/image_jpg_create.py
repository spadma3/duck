


def d8_compressed_image_from_cv_image(image_cv):
    """ 
        Create CompressedIamge from a CV image.
    
        TODO: assumptions on format?
    """
    import cv2
    import numpy as np
    from sensor_msgs import CompressedImage  # @UnresolvedImport
    compress =  cv2.imencode('.jpg', image_cv)[1]
    jpg_data = np.array(compress).tostring()
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = jpg_data
    return msg
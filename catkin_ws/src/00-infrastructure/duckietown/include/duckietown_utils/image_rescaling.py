from collections import OrderedDict
import numpy as np


def d8_image_zoom_linear(cv_image, ratio=4):
    """ Zooms up by the given ratio """
    import cv2
    H, W, _ = cv_image.shape
    W2 = int(W*ratio)
    H2 = int(H*ratio)
    res = cv2.resize(cv_image, (W2, H2), interpolation=cv2.INTER_NEAREST)
    return res

def d8_image_resize_no_interpolation(cv_image, new_shape):
    """
        new_shape = (H, W)
    """
    import cv2
    H, W = new_shape
    res = cv2.resize(cv_image, (W,H), interpolation=cv2.INTER_NEAREST)
    return res


def resize_small_images(image_dict):
    assert isinstance(image_dict, dict)
    max_H, max_W = 0, 0
    for image in image_dict.values():
        H, W = image.shape[0:2]    
        max_H = max(max_H, W)
        max_W = max(max_W, W)
        
    d = OrderedDict()
    for k, image in image_dict.items():
        H, W = image.shape[0:2]
        ratio = max(max_H*1.0/H, max_W*1.0/W)
        ratio = int(np.ceil(ratio))
        if ratio > 1:
            image2 = d8_image_zoom_linear(image, ratio)
        else:
            image2 = image
        d[k] = image2
    return d


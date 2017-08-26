import cv2


def d8_image_zoom_linear(cv_image, ratio=4):
    """ Zooms up by the given ratio """
    H, W, _ = cv_image.shape
    res = cv2.resize(cv_image, (W*ratio, H*ratio), interpolation=cv2.INTER_NEAREST)
    return res
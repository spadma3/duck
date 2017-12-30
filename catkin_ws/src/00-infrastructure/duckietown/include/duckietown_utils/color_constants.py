

class ColorConstants(object):
    STR_WHITE = 'white'
    STR_YELLOW = 'yellow'
    STR_RED = 'red'
    STR_BLACK = 'black'
    STR_GRAY = 'gray'
    STR_GREEN = 'green'
    STR_BLUE = 'blue'
    
    BLACK = (0,0,0) # XXX
    BGR_RED = (0,0,255)
    BGR_GREEN = (0,255,0)
    BGR_WHITE = (255,255,255)
    BGR_BLACK = (0,0,0)
    BGR_GRAY = (128,128,128)
    BGR_BLUE = (255,0,0)
    BGR_YELLOW = (0, 255,255)

def bgr_color_from_string(s):
    d = {
        ColorConstants.STR_YELLOW: ColorConstants.BGR_YELLOW,
        ColorConstants.STR_WHITE: ColorConstants.BGR_WHITE,
        ColorConstants.STR_BLACK: ColorConstants.BGR_BLACK,
        ColorConstants.STR_BLUE: ColorConstants.BGR_BLUE,
        ColorConstants.STR_RED: ColorConstants.BGR_RED,
        ColorConstants.STR_GRAY: ColorConstants.BGR_GRAY,
        ColorConstants.STR_GREEN: ColorConstants.BGR_GREEN,
    }
    if not s in d:
        msg = 'No color %r found in %s' % (s, list(d))
        raise ValueError(msg)
    return d[s]



class ColorConstants(object):
    STR_WHITE = 'white'
    STR_YELLOW = 'yellow'
    STR_RED = 'red'
    STR_BLACK = 'black'
    STR_GRAY = 'gray'
    
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
        'yellow': ColorConstants.BGR_YELLOW,
        'white': ColorConstants.BGR_WHITE,
        'black': ColorConstants.BGR_BLACK,
        'blue': ColorConstants.BGR_BLUE,
        'red': ColorConstants.BGR_RED,
        'gray': ColorConstants.BGR_GRAY,
    }
    if not s in d:
        msg = 'No color %r found in %s' % (s, list(d))
        raise ValueError(msg)
    return d[s]

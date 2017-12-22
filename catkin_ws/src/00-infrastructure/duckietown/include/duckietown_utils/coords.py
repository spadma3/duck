import numpy as np
from geometry.poses import SE2_from_xytheta

# Any time we use the variable "xytheta" it is a numpy array
# with the following datatype
DATATYPE_XYTHETA =  np.dtype([('x', 'float64'), ('y', 'float64'), ('theta', 'float64')])

def xytheta_from_xyth(xyth):
    return np.array([xyth['x'], xyth['y'], xyth['theta']])

def SE2_from_xyth(xyth):
    xytheta = xytheta_from_xyth(xyth)
    return SE2_from_xytheta(xytheta)
import numpy as np


# Any time we use the variable "xytheta" it is a numpy array
# with the following datatype
# # DATATYPE_XYTHETA =  np.dtype([('x', 'float64'), ('y', 'float64'), ('theta', 'float64')])
# 
# def xytheta_from_xyth(xyth):
#     return np.array([xyth['x'], xyth['y'], xyth['theta']])
# 
# def SE2_from_xyth(xyth):
#     from geometry.poses import SE2_from_xytheta
#     xytheta = xytheta_from_xyth(xyth)
#     return SE2_from_xytheta(xytheta)


# meters from inches
m_from_in = lambda x: x * 0.0254 


def norm_angle(theta):
    while theta < -np.pi:
        theta += np.pi*2
    
    while theta > +np.pi:
        theta -= np.pi*2
    
    assert -np.pi <= theta <= +np.pi
    
    return theta
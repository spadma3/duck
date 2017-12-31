import numpy as np

__all__ = [
    'norm_angle',
    'm_from_in',
]


# meters from inches
m_from_in = lambda x: x * 0.0254 


def norm_angle(theta):
    if np.isinf(theta) or np.isnan(theta):
        msg = 'Invalid value for theta: %s' % theta
        raise ValueError(msg)
    
    while theta < -np.pi:
        theta += np.pi*2
    
    while theta > +np.pi:
        theta -= np.pi*2
    
    assert -np.pi <= theta <= +np.pi
    
    return theta
from contextlib import contextmanager
import time
from .logging_logger import logger

__all__ = [
    'rospy_timeit_clock',
    'rospy_timeit_wall',
    'timeit_clock',
]

@contextmanager
def rospy_timeit_clock(s):
    import rospy
    t0 = time.clock()
    yield
    delta = time.clock() - t0    
    rospy.loginfo('%10d ms: %s' % ( 1000*delta, s))

@contextmanager
def rospy_timeit_wall(s):
    import rospy
    t0 = time.time()
    yield
    delta = time.time() - t0    
    rospy.loginfo('%10d ms: %s' % ( 1000*delta, s))
    

@contextmanager
def timeit_clock(desc, minimum=None):
#     logger.debug('timeit %s ...' % desc)
    t0 = time.clock()
    yield
    t1 = time.clock()
    delta = t1 - t0
    if minimum is not None:
        if delta < minimum:
            return
#     logger.debug('timeit result: %.2f s (>= %s) for %s' % (delta, minimum, desc))
    logger.debug('timeit_clock: %6.1f ms (>= %s) for %s' % (delta*1000, minimum, desc))

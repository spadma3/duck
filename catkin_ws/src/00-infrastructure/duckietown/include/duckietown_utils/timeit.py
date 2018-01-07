from contextlib import contextmanager
import time

from .constants import DuckietownConstants
from .logging_logger import logger

__all__ = [
    'rospy_timeit_clock',
    'rospy_timeit_wall',
    'timeit_clock',
    'timeit_wall',
]


@contextmanager
def rospy_timeit_clock(s):
    import rospy
    t0 = time.clock()
    yield
    delta = time.clock() - t0
    rospy.loginfo('%10d ms: %s' % (1000 * delta, s))


@contextmanager
def rospy_timeit_wall(s):
    import rospy
    t0 = time.time()
    yield
    delta = time.time() - t0
    rospy.loginfo('%10d ms: %s' % (1000 * delta, s))


class Stack:
    stack = []


@contextmanager
def timeit_generic(desc, minimum, time_function):
#     logger.debug('timeit %s ...' % desc)
    t0 = time_function()
    Stack.stack.append(desc)
    yield
    Stack.stack.pop()
    t1 = time_function()
    delta = t1 - t0
    if minimum is not None:
        if delta < minimum:
            return
#     logger.debug('timeit result: %.2f s (>= %s) for %s' % (delta, minimum, desc))

    if DuckietownConstants.show_timeit_benchmarks or minimum is not None:
        pre = '   ' * len(Stack.stack)
    #     logger.debug('timeit_clock: %s %6.1f ms (>= %s) for %s' % (pre, delta*1000, minimum, desc))
        msg = 'timeit_clock: %s %6.2f ms  for %s' % (pre, delta * 1000, desc)
        logger.debug(msg)

#        import rospy  # XXX
#        rospy.loginfo(msg)


@contextmanager
def timeit_clock(desc, minimum=None):
    with timeit_generic(desc=desc, minimum=minimum, time_function=time.clock) as f:
        yield f


@contextmanager
def timeit_wall(desc, minimum=None):
    with timeit_generic(desc=desc, minimum=minimum, time_function=time.time) as f:
        yield f


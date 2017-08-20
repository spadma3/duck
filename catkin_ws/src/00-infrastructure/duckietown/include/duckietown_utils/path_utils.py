import os

from duckietown_utils.exceptions import DTConfigException
import rospkg


def expand_all(filename):
    """
        Expands ~ and ${ENV} in the string. 
        
        Raises DTConfigException if some environment variables
        are not expanded.
        
    """
    fn = filename
    fn = os.path.expanduser(fn)
    fn = os.path.expandvars(fn)
    if '$' in fn:
        msg = 'Could not expand all variables in path %r.' % fn
        raise DTConfigException(msg)
    return fn


def get_ros_package_path(package_name):
    """ Returns the path to a package. """
    rospack = rospkg.RosPack()
    return rospack.get_path(package_name)


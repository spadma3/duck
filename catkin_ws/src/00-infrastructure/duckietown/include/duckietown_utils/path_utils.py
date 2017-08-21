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


def display_filename(filename):
    """ Displays a filename in a possibly simpler way """
    cwd = os.path.realpath(os.getcwd())
    filename = os.path.realpath(filename)
    if filename.startswith(cwd+'/'):
        filename = os.path.relpath(filename, cwd)
        return filename
    
    return filename
    
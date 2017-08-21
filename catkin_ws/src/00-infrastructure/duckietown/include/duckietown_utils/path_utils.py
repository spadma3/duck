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
    rospack = rospkg.RosPack()  # @UndefinedVariable
    return rospack.get_path(package_name)


def display_filename(filename):
    """ Displays a filename in a possibly simpler way """
    cwd = os.path.realpath(os.getcwd())
    filename = os.path.realpath(filename)
    from duckietown_utils.constants import get_catkin_ws_src
    cw = os.path.realpath(get_catkin_ws_src())
    if filename.startswith(cw):
        return '${CATKIN_WS}/' + filename[len(cw)+1:]
    
    if filename.startswith(cwd+'/'):
#         print('cwd %s filename %s' % (cwd, filename))
        filename = os.path.relpath(filename, cwd +'/.')
        return filename
    
    return filename
    
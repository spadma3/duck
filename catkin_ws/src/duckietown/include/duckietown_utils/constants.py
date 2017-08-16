from duckietown_utils.path_utils import expand_all
from duckietown_utils.exceptions import DTConfigException
import os

class DuckietownConstants():
    DUCKIETOWN_ROOT_variable = 'DUCKIETOWN_ROOT'
    DUCKIEFLEET_ROOT_variable = 'DUCKIEFLEET_ROOT'
    
    # inside DUCKIEFLEET_ROOT
    scuderia_filename = 'scuderia.yaml'
    
    
def get_duckietown_root():
    """ Returns the path of DUCKIETOWN_ROOT and checks it exists """
    return _get_dir(DuckietownConstants.DUCKIETOWN_ROOT_variable)

def get_duckiefleet_root():
    """ Returns the path of DUCKIETOWN_ROOT and checks it exists """
    return _get_dir(DuckietownConstants.DUCKIEFLEET_ROOT_variable)

def get_machines_files_path():
    ''' Gets the path to the machines files. It might not exist. '''
    duckietown_root = get_duckietown_root()
    machines = os.path.join(duckietown_root, 'catkin_ws/src/duckietown/machines')
    return machines

def get_scuderia_path():
    ''' Gets the path to the scuderia files. It might not exist. '''
    duckiefleet = get_duckiefleet_root()
    machines = os.path.join(duckiefleet, DuckietownConstants.scuderia_filename)
    return machines

def _get_dir(variable_name):
    if not variable_name in os.environ:
        msg = 'Environment variable %r not defined.' % variable_name
        raise DTConfigException(msg)
    
    fn = expand_all(os.environ[variable_name])
    
    if not os.path.exists(fn):
        msg = 'Could not get %s: dir does not exist: %s' % (variable_name, fn)
        raise DTConfigException(msg)
    
    return fn
    
import os

from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.path_utils import expand_all


class DuckietownConstants():
    DUCKIETOWN_ROOT_variable = 'DUCKIETOWN_ROOT'
    DUCKIEFLEET_ROOT_variable = 'DUCKIEFLEET_ROOT'
    
    # inside DUCKIEFLEET_ROOT
    scuderia_filename = 'scuderia.yaml'
    machines_path_rel_to_root = 'catkin_ws/src/00-infrastructure/duckietown/machines'
    
    enforce_no_tabs = True
    enforce_naming_conventions = True
    
    # The rules for good readmes do not apply to these packages
    good_readme_exceptions = ['apriltags_ros', 'apriltags', 'duckietown', 'isam']

    
def get_duckietown_root():
    """ Returns the path of DUCKIETOWN_ROOT and checks it exists """
    return _get_dir(DuckietownConstants.DUCKIETOWN_ROOT_variable)

def get_duckiefleet_root():
    """ Returns the path of DUCKIETOWN_ROOT and checks it exists """
    return _get_dir(DuckietownConstants.DUCKIEFLEET_ROOT_variable)

def get_machines_files_path():
    ''' Gets the path to the machines files. It might not exist. '''
    duckietown_root = get_duckietown_root()
    machines = os.path.join(duckietown_root, DuckietownConstants.machines_path_rel_to_root)
    return machines

def get_catkin_ws_src():
    ''' Returns the path to the src/ dir in catkin_ws '''
    duckietown_root = get_duckietown_root()
    machines = os.path.join(duckietown_root, 'catkin_ws/src')
    return machines

def get_list_of_packages_in_catkin_ws():
    """
        Returns an ordered dictionary <package name>: <package dir>
        of packages that exist in catkin_ws/src.
    
        Raises DTConfigException if $DUCKIETOWN_ROOT is not set.
    """
    src = get_catkin_ws_src()
    package_files = locate_files(src, 'package.xml')
    results = {}
    for p in package_files:
        dn = os.path.dirname(p)
        entry = os.path.basename(dn)
        results[entry] = dn
    # We expect at least these two packages
    if not 'duckietown' in results:
        raise ValueError('Could not find duckietown')
    if not 'what_the_duck' in results:
        raise ValueError('Could not find what_the_duck') 
    return results
    

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
    
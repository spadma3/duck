from collections import OrderedDict
import os

from contracts.utils import check_isinstance
from ruamel import yaml
import ruamel.yaml
from ruamel.yaml.error import YAMLError

from duckietown_utils import logger
from duckietown_utils.constants import get_catkin_ws_src, get_duckiefleet_root,\
    get_duckietown_data, get_duckietown_local_log_downloads
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.friendly_path_imp import friendly_path
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.system_cmd_imp import contract


def interpret_yaml_file(filename, contents, f):
    """ 
        f is a function that takes
        
            f(filename, data)
            
        f can raise KeyError, or DTConfigException """
    try:
        try:
            data = yaml.load(contents, Loader=ruamel.yaml.Loader)
        except YAMLError as e:
            msg = 'Invalid YAML content:'
            raise_wrapped(DTConfigException, e, msg, compact=True)

        try:  
            return f(filename, data)
        except KeyError as e:
            msg = 'Missing field "%s".' % e.args[0]
            raise DTConfigException(msg)
     
    except DTConfigException as e:
        msg = 'Could not interpret the contents of the file using %s()\n' % f.__name__
        msg += '   %s\n' % friendly_path(filename)
        msg += 'Contents:\n' + indent(contents, ' > ')
        raise_wrapped(DTConfigException, e, msg, compact=True) 

def get_config_sources():
    
    sources = []
    # We look in $DUCKIETOWN_ROOT/catkin_ws/src
    sources.append(get_catkin_ws_src())
    # then we look in $DUCKIETOWN_FLEET
    sources.append(get_duckiefleet_root())
    
    return sources
 
@contract(pattern=str, sources='seq(str)')
def look_everywhere_for_config_files(pattern, sources):
    """
        Looks for all the configuration files by the given pattern.    
        Returns a dictionary filename -> contents.
    """
    check_isinstance(sources, list)
    
    logger.debug('Reading configuration files with pattern %s.' % pattern)
 
    results = OrderedDict()
    for s in sources:
        filenames = locate_files(s, pattern)
        for filename in filenames:
            contents = open(filename).read()
            results[filename] = contents
        logger.debug('%4d files found in %s' % (len(results), friendly_path(s)))
    return results

def look_everywhere_for_bag_files(pattern='*.bag'):
    """
        Looks for all the bag files    
        Returns a list of basename -> filename.
    """
    sources = []
    # We look in $DUCKIETOWN_ROOT/catkin_ws/src
    sources.append(get_catkin_ws_src())
    # then we look in $DUCKIETOWN_FLEET
    sources.append(get_duckiefleet_root())
    sources.append(get_duckietown_data())
    # downloads 
    p = get_duckietown_local_log_downloads()
    if os.path.exists(p):
        sources.append(p)
    
    logger.debug('Looking for files with pattern %s...' % pattern)
    
    results = OrderedDict()
    for s in sources:
        filenames = locate_files(s, pattern)
        logger.debug('%5d files in %s' % (len(filenames), friendly_path(s)))
        for filename in filenames:
            basename, _ = os.path.splitext(os.path.basename(filename))
            if basename in results:
                one = filename
                two = results[basename]
                if not same_file_content(one, two):
                    msg = 'Two bags with same name but different content:\n%s\n%s' %(one, two)
                    raise DTConfigException(msg)
                else:
                    msg = 'Two copies of bag found:\n%s\n%s' %(one, two)
                    logger.warn(msg)
                    continue
            results[basename] = filename
    return results

def same_file_content(a, b):
    """ Just check the size """
    s1 = os.stat(a).st_size
    s2 = os.stat(b).st_size
    return s1==s2

    

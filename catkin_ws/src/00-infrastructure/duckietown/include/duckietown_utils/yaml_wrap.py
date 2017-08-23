from collections import OrderedDict

from ruamel import yaml
from ruamel.yaml.error import YAMLError

from duckietown_utils import logger
from duckietown_utils.constants import get_catkin_ws_src, get_duckiefleet_root,\
    get_duckietown_data
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.path_utils import display_filename
import os


def interpret_yaml_file(filename, contents, f):
    """ 
        f is a function that takes
        
            f(filename, data
        f can raise KeyError, or DTConfigException """
    try:
        try:
            data = yaml.load(contents)
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
        msg += '   %s\n' % display_filename(filename)
        msg += 'Contents:\n' + indent(contents, ' > ')
        raise_wrapped(DTConfigException, e, msg, compact=True) 
        
def look_everywhere_for_config_files(pattern):
    """
        Looks for all the configuration files by the given pattern.    
        Returns a dictionary filename -> contents.
    """
    sources = []
    # We look in $DUCKIETOWN_ROOT/catkin_ws/src
    sources.append(get_catkin_ws_src())
    # then we look in $DUCKIETOWN_FLEET
    sources.append(get_duckiefleet_root())
    
    logger.info('Reading configuration files from sources:\n'+'\n'.join(sources))

    results = OrderedDict()
    for s in sources:
        filenames = locate_files(s, pattern)
        for filename in filenames:
            contents = open(filename).read()
            results[filename] = contents
    return results

def look_everywhere_for_bag_files():
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
    
    logger.info('Reading bag files from sources:\n'+'\n'.join(sources))
    pattern = '*.bag'
    results = OrderedDict()
    for s in sources:
        filenames = locate_files(s, pattern)
        for filename in filenames:
            basename, _ = os.path.splitext(os.path.basename(filename))
            if basename in results:
                one = filename
                two = results[basename]
                msg = 'Two bags with same file:\n%s\n%s' %(one, two)
                raise DTConfigException(msg)
            results[basename] = filename
    return results

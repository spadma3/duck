from collections import namedtuple
import os

from frozendict import frozendict
import yaml
from yaml.error import YAMLError

from duckietown_utils import logger
from duckietown_utils.constants import get_catkin_ws_src, get_duckiefleet_root
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.locate_files_impl import locate_files


SUFFIX = '.config.yaml'

ConfigInfo = namedtuple('ConfigInfo', 
        'filename package_name node_name config_name date_effective description extends values '
        'valid error_if_invalid ')

 
def get_configuration_files(package_name, node_name):
    """
        Gets all the configuration files for easynodes.
        
        These are of the form
        
            package-node.![name].config.yaml
        
        For timing:
            
            package-node.![name].201XMMDD.config.yaml
    """
    
def get_all_configuration_files():
    configs = search_all_configuration_files()
    results = []
    for filename in configs:
        c = interpret_config_file(filename)
        results.append(c)
    return results



def search_all_configuration_files():
    sources = []
    # We look in $DUCKIETOWN_ROOT/catkin_ws/src
    sources.append(get_catkin_ws_src())
    # then we look in $DUCKIETOWN_FLEET
    sources.append(get_duckiefleet_root())
    
    configs = []
    pattern = '*' + SUFFIX
    logger.info('Looking for %s files.' % pattern) 
    for s in sources:
        fs = locate_files(s, pattern)
        logger.info('%4d files in %s' % (len(fs), s))
        configs.extend(fs)
    
#     logger.debug('I found:\n' + "\n".join(configs))
    
    return configs


def interpret_config_file(filename):
    """ 
        Returns a ConfigInfo.     
    """
    try:
        basename = os.path.basename(filename)
        base = basename.replace(SUFFIX, '')
        # now we have something like
        #   package-node.config_name.date
        # or
        #   package-node.config_name
        if not '.' in base:
            msg = 'Invalid filename %r.' % filename
            raise DTConfigException(msg)
        
        tokens = base.split('.')
        if len(tokens) > 3:
            msg = 'Too many periods/tokens (tokens=%s)' % tokens
            raise DTConfigException(msg)
        
        if len(tokens) <= 2:
            #  package-node.config_name
            package_node = tokens[0]
            if not '-' in package_node:
                msg = 'Expected a "-" in "%s".' % package_node
                raise DTConfigException(msg)
            i = package_node.index('-')
            package_name = package_node[:i]
            node_name  = package_node[i+1:]
        
        config_name = tokens[1]
        
        if len(tokens) == 3:
            # package-node.config_name.date
            date_effective = tokens[2]
        else:
            date_effective = '20170101'
        from dateutil.parser import parse

        try:
            date_effective  = parse(date_effective)
        except:
            msg = 'Cannot interpret "%s" as a date.' % date_effective
            raise DTConfigException(msg)

        # now read file
        
        contents = open(filename).read()
        try:
            try:
                data = yaml.load(contents)
            except YAMLError as e:
                raise_wrapped(DTConfigException, e, 'Invalid YAML', compact=True)
                
            if not isinstance(data, dict):
                msg = 'Expected a dictionary inside.'
                raise DTConfigException(msg)
            
            for field in ['description', 'values']:
                if not field in data:
                    msg = 'Missing field "%s".' % field
                    raise DTConfigException(msg)
                
            description = data.pop('description')
            if not isinstance(description, str):
                msg = 'I expected that "description" is a string, obtained %r.' % description 
                raise DTConfigException(msg)
            
            extends = data.pop('extends', [])
            if not isinstance(extends, list):
                msg = 'I expected that "extends" is a list, obtained %r.' % extends
                raise DTConfigException(msg)
            
            values = data.pop('values')
            if not isinstance(values, dict):
                msg = 'I expected that "values" is a dictionary, obtained %s.' % type(values) 
                raise DTConfigException(msg)
            
            # Freeze the data
            extends = tuple(extends)
            values = frozendict(values)
            
        except DTConfigException as e:
            msg = 'Could not interpret the contents of the file\n'
            msg += '   %s\n' % friendly_path(filename)
            msg += 'Contents:\n' + indent(contents, ' > ')
            raise_wrapped(DTConfigException, e, msg, compact=True) 
        
        return ConfigInfo(filename=filename, package_name=package_name, node_name=node_name, 
                          config_name=config_name,
                          date_effective=date_effective,extends=extends,
                          description=description, values=values,
                          # not decided
                          valid=None,
                          error_if_invalid=None)
    
    except DTConfigException as e:
        msg = 'Invalid file %s' % friendly_path(filename)
        raise_wrapped(DTConfigException, e, msg, compact=True)

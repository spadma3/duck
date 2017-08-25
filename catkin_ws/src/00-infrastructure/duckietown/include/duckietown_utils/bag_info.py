import os
import subprocess

import yaml

from duckietown_utils import logger

from .caching import get_cached
from .instantiate_utils import indent


__all__ = ['rosbag_info', 'rosbag_info_cached']

def rosbag_info_cached(filename):
    def f():
        return rosbag_info(filename)
    basename = os.path.basename(filename)
    cache_name = 'rosbag_info/' + basename
    return get_cached(cache_name, f, quiet=True)
    
    

def rosbag_info(bag):
    stdout = subprocess.Popen(['rosbag', 'info', '--yaml', bag],
                              stdout=subprocess.PIPE).communicate()[0]
    try:
        info_dict = yaml.load(stdout)
    except:
        logger.error('Could not parse yaml:\n%s' % indent(stdout, '| '))
        raise
    return info_dict
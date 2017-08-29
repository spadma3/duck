from collections import OrderedDict
import os

from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.system_cmd_imp import contract
from duckietown_utils.yaml_pretty import yaml_dump_pretty
from easy_regression.conditions.result_db import ResultDBEntry


@contract(r=ResultDBEntry)
def yaml_from_rdbe(r):
    d = OrderedDict()
    d['description'] = 'The result of running a unit test'
    d['constructor'] = 'easy_regression.rdbe_from_yaml'
    d['parameters']= r._asdict()
    return yaml_dump_pretty(d)

def rdbe_from_yaml(**parameters):
    return ResultDBEntry(**parameters)

def get_unique_filename(rt_name, rdbe):
    commit = rdbe.commit[-8:]    
    d =rdbe.date.replace('-','')
    basename = rt_name + '_%s_%s_%s.rdbe.yaml' % (d, rdbe.branch, commit)
    
    dr = get_ros_package_path('easy_regression')
    filename = os.path.join(dr, 'db', rt_name, basename)
    return filename

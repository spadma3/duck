from duckietown_utils.caching import get_cached
from duckietown_utils.yaml_wrap import look_everywhere_for_bag_files
from easy_logs.logs_structure import PhysicalLog
import os
from ruamel import yaml
import re


def get_easy_logs_db():
    if EasyLogsDB._singleton is None:
        EasyLogsDB._singleton = get_cached('EasyLogsDB', EasyLogsDB)
    return EasyLogsDB._singleton

class EasyLogsDB():
    _singleton = None 
     
    
    def __init__(self):
        self.logs = load_all_logs()
        
from procgraph_ros.bag_utils import rosbag_info  # @UnresolvedImport
 
    
def rosbag_info_cached(filename):
    def f():
        return rosbag_info(filename)
    basename = os.path.basename(filename)
    cache_name = 'rosbag_info/' + basename
    return get_cached(cache_name, f)
    
    
def read_stats(pl):
    assert isinstance(pl, PhysicalLog)
    
    info = rosbag_info_cached(pl.filename)
    
    # print yaml.dump(info)
    length = info['duration']
    date_ms = info['start']
    date = date_ms
    try:
        vehicle_name = which_robot(info) 
    except ValueError as e:
        vehicle_name = None
        
    return pl._replace(date=date, length=length, vehicle_name=vehicle_name,
                       bag_info=info)

def which_robot(info):
    pattern  = r'/(\w+)/camera_node/image/compressed'
    for topic in info['topics']:
        m = re.match(pattern, topic['topic'])
        if m: 
            vehicle_name = m.group(1)
            return vehicle_name
    msg = 'Could not find a topic matching %s' % pattern
    raise ValueError(msg)

def load_all_logs(which):
    pattern = which + '.bag'
    basename2filename = look_everywhere_for_bag_files(pattern=pattern)
    logs = []
    for basename, filename in basename2filename.items():
        log_name = basename  
        date = None
        size=  os.stat(filename).st_size
        
        l = PhysicalLog(log_name=log_name, 
                        map_name=None, 
                        description=None, 
                        length=None,
                        date=date,  
                        size=size,
                        has_camera=None,
                        vehicle_name = None,
                        filename=filename,
                        bag_info=None)
        l = read_stats(l)
        logs.append(l)
    
    return logs
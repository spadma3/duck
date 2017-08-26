from collections import OrderedDict
import os
import re

from contracts.utils import check_isinstance

from duckietown_utils.bag_info import rosbag_info_cached
from duckietown_utils.caching import get_cached
from duckietown_utils.constants import DuckietownConstants
from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.yaml_wrap import look_everywhere_for_bag_files
from easy_logs.logs_structure import PhysicalLog


from duckietown_utils.fuzzy import fuzzy_match


def get_urls_path():
    d = get_ros_package_path('easy_logs')
    f = os.path.join(d, 'dropbox.urls.yaml')
    return f


def get_easy_logs_db():
    if EasyLogsDB._singleton is None:
        f = EasyLogsDB
        use_cache = DuckietownConstants.use_cache_for_logs
        EasyLogsDB._singleton = get_cached('EasyLogsDB', f) if use_cache else f()
    return EasyLogsDB._singleton


class EasyLogsDB():
    _singleton = None

    def __init__(self):
        # ordereddict str -> PhysicalLog
        self.logs = load_all_logs()
         
    def query(self, query, raise_if_no_matches=True):
        """
            query: a string

            Returns an OrderedDict str -> PhysicalLog.
        """
        check_isinstance(query, str)
        result = fuzzy_match(query, self.logs, raise_if_no_matches=raise_if_no_matches)
        return result


def read_stats(pl):
    assert isinstance(pl, PhysicalLog)

    info = rosbag_info_cached(pl.filename)
    if info is None:
        return pl._replace(valid=False, error_if_invalid='Not indexed')

    # print yaml.dump(info)
    length = info['duration']
    if length is None:
        return pl._replace(valid=False, error_if_invalid='Empty bag.')

    date_ms = info['start']
    if date_ms < 156600713:
        return pl._replace(valid=False, error_if_invalid='Date not set.')

    date = date_ms

    pl = pl._replace(date=date, length=length, bag_info=info)

    try:
        vehicle = which_robot(info)
        pl =pl._replace(vehicle=vehicle, has_camera=True)
    except ValueError:
        vehicle = None
        pl = pl._replace(valid=False, error_if_invalid='No camera data.')

#     camera_topic = '/%s/camera_node/image/compressed' % vehicle
#
#     found = False
#     for _ in info['topics']:
#         if _['topic'] == camera_topic:
#             found = True
#
#     if not found:
#

    return pl

def which_robot(info):
    pattern  = r'/(\w+)/camera_node/image/compressed'
    for topic in info['topics']:
        m = re.match(pattern, topic['topic'])
        if m:
            vehicle = m.group(1)
            return vehicle
    msg = 'Could not find a topic matching %s' % pattern
    raise ValueError(msg)

def load_all_logs(which='*'):
    pattern = which + '.bag'
    basename2filename = look_everywhere_for_bag_files(pattern=pattern)
    logs = OrderedDict()
    for basename, filename in basename2filename.items():
        log_name = basename
        date = None
        size =  os.stat(filename).st_size

        l = PhysicalLog(log_name=log_name,
                        map_name=None,
                        description=None,
                        length=None,
                        date=date,
                        size=size,
                        has_camera=None,
                        vehicle = None,
                        filename=filename,
                        bag_info=None,
                        valid=True,
                        error_if_invalid=None)
        l = read_stats(l)
        logs[l.log_name]= l

    return logs

from collections import OrderedDict
import copy
import os

import duckietown_utils as dtu

from .logs_structure import PhysicalLog
from .time_slice import filters_slice


def get_easy_logs_db():
    return get_easy_logs_db_cached_if_possible()


def get_easy_logs_db_cached_if_possible():
    if EasyLogsDB._singleton is None:
        f = EasyLogsDB
        EasyLogsDB._singleton = dtu.get_cached('EasyLogsDB', f)

        fn = os.path.join(dtu.get_duckietown_root(), 'caches', 'candidate_cloud.yaml')

        if not os.path.exists(fn):
            logs = copy.deepcopy(EasyLogsDB._singleton.logs)
            # remove the field "filename"
            for k, v in logs.items():
                logs[k] = v._replace(filename=None)

            dtu.yaml_write_to_file(logs, fn)

    return EasyLogsDB._singleton


def get_easy_logs_db_fresh():
    if EasyLogsDB._singleton is None:
        f = EasyLogsDB
        EasyLogsDB._singleton = f()
    return EasyLogsDB._singleton


def get_easy_logs_db_cloud():
    cloud_file = dtu.require_resource('cloud.yaml')

#     cloud_file = os.path.join(get_ros_package_path('easy_logs'), 'cloud.yaml')
#     if not os.path.exists(cloud_file):
#         url = "https://www.dropbox.com/s/vdl1ej8fihggide/duckietown-cloud.yaml?dl=1"
#         download_url_to_file(url, cloud_file)

    with dtu.timeit_wall("loading DB"):
        dtu.logger.info('Loading cloud DB %s' % dtu.friendly_path(cloud_file))
        logs = dtu.yaml_load_file(cloud_file, plain_yaml=True)

    logs = OrderedDict(logs)
    dtu.logger.info('Loaded cloud DB with %d entries.' % len(logs))

    return EasyLogsDB(logs)


class EasyLogsDB(object):
    _singleton = None

    def __init__(self, logs=None):
        # ordereddict str -> PhysicalLog
        if logs is None:
            logs = load_all_logs()
        else:
            dtu.check_isinstance(logs, OrderedDict)

        # Let's get rid of these redundant logs from 2016
        for k in list(logs):
            if 'RCDP' in k:
                del logs[k]
        self.logs = logs

    @dtu.contract(returns=OrderedDict, query='str|list(str)')
    def query(self, query, raise_if_no_matches=True):
        """
            query: a string or a list of strings

            Returns an OrderedDict str -> PhysicalLog.

            The query can also be a filename.

        """
        if isinstance(query, list):
            res = OrderedDict()
            for q in query:
                res.update(self.query(q, raise_if_no_matches=False))
            if raise_if_no_matches and not res:
                msg = "Could not find any match for the queries:"
                for q in query:
                    msg += '\n- %s' % q
                raise dtu.DTNoMatches(msg)
            return res
        else:
            dtu.check_isinstance(query, str)

            filters = OrderedDict()
            filters.update(filters_slice)
            filters.update(dtu.filters0)
            result = dtu.fuzzy_match(query, self.logs, filters=filters,
                                 raise_if_no_matches=raise_if_no_matches)
            return result


def read_stats(pl):
    assert isinstance(pl, PhysicalLog)

    info = dtu.rosbag_info_cached(pl.filename)
    if info is None:
        return pl._replace(valid=False, error_if_invalid='Not indexed')

    # print yaml.dump(info)
    length = info['duration']
    if length is None:
        return pl._replace(valid=False, error_if_invalid='Empty bag.')

    pl = pl._replace(length=length, t0=0, t1=length, bag_info=info)

    try:
        vehicle = which_robot_from_bag_info(info)
        pl = pl._replace(vehicle=vehicle, has_camera=True)
    except ValueError:
        vehicle = None
        pl = pl._replace(valid=False, error_if_invalid='No camera data.')

    date_ms = info['start']
    if date_ms < 156600713:
        pl = pl._replace(valid=False, error_if_invalid='Date not set.')
    else:
        date = dtu.format_time_as_YYYY_MM_DD(date_ms)
        pl = pl._replace(date=date)

    return pl


def which_robot_from_bag_info(info):
    import re
    pattern = r'/(\w+)/camera_node/image/compressed'
    for topic in info['topics']:
        m = re.match(pattern, topic['topic'])
        if m:
            vehicle = m.group(1)
            return vehicle
    msg = 'Could not find a topic matching %s' % pattern
    raise ValueError(msg)


def is_valid_name(basename):
    forbidden = [',', '(', 'conflicted', ' ']
    for f in forbidden:
        if f in basename:
            return False
    return True


def load_all_logs(which='*'):
    pattern = which + '.bag'
    basename2filename = dtu.look_everywhere_for_bag_files(pattern=pattern)
    logs = OrderedDict()
    for basename, filename in basename2filename.items():
        if not is_valid_name(basename):
            msg = 'Ignoring Bag file with invalid file name "%r".' % (basename)
            msg += '\n Full path: %s' % filename
            dtu.logger.warn(msg)
            continue
        l = physical_log_from_filename(filename)

        logs[l.log_name] = l

    return logs


def physical_log_from_filename(filename):
    date = None
    size = os.stat(filename).st_size
    b = os.path.basename(filename)
    log_name, bagext = os.path.splitext(b)
    if bagext != '.bag':
        raise Exception(bagext)
    l = PhysicalLog(log_name=log_name,
                    map_name=None,
                    description=None,
                    length=None,
                    t0=None, t1=None,
                    date=date,
                    size=size,
                    has_camera=None,
                    vehicle=None,
                    filename=filename,
                    bag_info=None,
                    valid=True,
                    error_if_invalid=None)
    l = read_stats(l)
    return l

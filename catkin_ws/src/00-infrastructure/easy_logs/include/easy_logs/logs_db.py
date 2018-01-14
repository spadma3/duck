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


@dtu.memoize_simple
def get_dir_list(dirname):
    return list(os.listdir(dirname))


@dtu.contract(returns=PhysicalLog, filename=str)
def physical_log_from_filename(filename):
    date = None
    size = os.stat(filename).st_size
    b = os.path.basename(filename)
    base, bagext = os.path.splitext(b)
    if bagext != '.bag':
        raise Exception(filename)

    def ignore_record(rname):
        forbidden = [' ',  #names with spaces,
                     'active.avi',
                     'bag.info.yaml',
                     'bag.info ',
                     'zip',
                     ".timestamps",
                     ".metadata.yaml",
                     ]
        for f in forbidden:
            if f in rname:
#                msg = 'Ignoring resource %s' % rname
#                dtu.logger.warning(msg)
                return True
#        dtu.logger.warning(rname)

        return False

    description = OrderedDict()
    dirname = os.path.dirname(filename)
    siblings = get_dir_list(dirname)
    resources = OrderedDict()
    for s in siblings:
        basedot = base + '.'
        if s.startswith(basedot):
            rest = s[len(basedot):]
#            print('rest: %s' % rest)
#            ndots = rest.count(".")
#            if ndots == 1:
#                record_name, rest_ext = os.path.splitext(rest)
#                print('rest: %s ' % record_name)

            record_name = rest.lower()
            fn = os.path.join(dirname, s)
            if not ignore_record(record_name):
                resources[record_name] = dtu.create_hash_url(fn)
#            else:
#                print('will not interpret %r' % s)
#    records = [
#        ('bag', '{base}.bag'),
#        ('external', '{base}.external.mp4'),
#        ('external', '{base}.external.MP4'),
##        ('video', '{log_name}.video.mp4'),
##        ('thumbnails', '{log_name}.thumbnails.png'),
##        ('info', '{log_name}.info.yaml'),
#    ]
#    rep = dict(base=base)
#
#    print(filename)
#    for record_name, pattern in records:
#        supposed0 = pattern.format(**rep)
#        supposed = os.path.join(dirname, supposed0)
#        if os.path.exists(supposed):
##            print(supposed)
#            resources[record_name] = dtu.create_hash_url(supposed)

#    print resources

    # at least the bag file should be present
    assert 'bag' in resources

    l = PhysicalLog(log_name=base,
#                    map_name=None,
                    resources=resources,
                    description=description,
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
    if l.bag_info is not None:
        start = l.bag_info['start']
        canonical = dtu.format_time_as_YYYYMMDDHHMMSS(start)

        if l.vehicle is not None:
            s = l.vehicle
            M = 8
            if len(s) > M:
                s = s[:M]
        else:
            s = 'unknown'
        canonical = canonical + '_' + s
        #print('canonical: %s' % canonical)
        l = l._replace(log_name=canonical)

    return l


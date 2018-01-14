from collections import OrderedDict, namedtuple
from collections import defaultdict
import copy
import os

import duckietown_utils as dtu
from duckietown_utils.memoization import memoize_simple

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


def _read_stats(pl, use_filename):
    assert isinstance(pl, PhysicalLog)

    info = dtu.rosbag_info_cached(use_filename)
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


def _get_base_base(x):
    if not '.' in x:
        msg = 'Invalid: %s' % x
        raise ValueError(msg)
    return x[:x.index('.')]


AllResources = namedtuple('AllResources',
                          'basename2filename base2basename2filename')


@dtu.memoize_simple
def get_all_resources():
    patterns = ['*.bag', '*.mp4',
                '*.jpg',
#                '*.png',
                '*.mov', '*.mts'
                ]
    basename2filename = dtu.look_everywhere_for_files(patterns=patterns, silent=True)
    base2basename2filename = defaultdict(lambda: {})
    for basename, fn in basename2filename.items():
        if 'ii-datasets' in basename:
            continue
        base = _get_base_base(basename)
#        print('basename: %s base: %s filename: %s' % (basename, base, fn))
        base2basename2filename[base][basename] = fn
    return AllResources(basename2filename=basename2filename,
                        base2basename2filename=base2basename2filename)


def load_all_logs():

    all_resources = get_all_resources()

    logs = OrderedDict()

    for basename, filename in all_resources.basename2filename.items():
        if not basename.endswith('.bag'):
            continue

        if not is_valid_name(basename):
            msg = 'Ignoring Bag file with invalid file name "%r".' % (basename)
            msg += '\n Full path: %s' % filename
            dtu.logger.warn(msg)
            continue

        base = _get_base_base(basename)

        if basename != base + '.bag':
            continue
#        print('basename: %s base: %s filename: %s related : %s' % (basename, base, filename,
#                                                      related))
        l = physical_log_from_filename(filename, all_resources.base2basename2filename)

        logs[l.log_name] = l

    return logs


@dtu.contract(returns=PhysicalLog, filename=str)
def physical_log_from_filename(filename, base2basename2filename):
    """

        related: basename -> filename
    """
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

        return False

    description = OrderedDict()

    resources = OrderedDict()

    l = PhysicalLog(log_name=base,  # might be replaced later
                    resources=resources,
                    description=description,
                    length=None,
                    t0=None, t1=None,
                    date=date,
                    size=size,
                    has_camera=None,
                    vehicle=None,
                    filename=None,
#                    filename=filename,
                    bag_info=None,
                    valid=True,
                    error_if_invalid=None)

    l = _read_stats(l, use_filename=filename)
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

    possible_bases = set()
    possible_bases.add(base)
    possible_bases.add(l.log_name)

    for _base in possible_bases:
        for s in base2basename2filename[_base]:
            basedot = _base + '.'
            if s.startswith(basedot):
                rest = s[len(basedot):]
                record_name = rest.lower()
                if not ignore_record(record_name):
                    fn = base2basename2filename[_base][s]
                    resources[record_name] = dtu.create_hash_url(fn)

    # at least the bag file should be present
    assert 'bag' in resources

    return l


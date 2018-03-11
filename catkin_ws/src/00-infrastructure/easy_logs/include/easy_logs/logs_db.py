from collections import OrderedDict, namedtuple, defaultdict
import copy
import os

import duckietown_utils as dtu
from duckietown_utils.yaml_pretty import yaml_dump_pretty

from .logs_structure import PhysicalLog
from .logs_structure import yaml_from_physical_log, \
    physical_log_from_yaml
from .resource_desc import create_dtr_version_1, DTR
from .time_slice import filters_slice


def get_easy_logs_db():
    return get_easy_logs_db_cached_if_possiblew(EasyLogsDB, write_candidate_cloud=False)


def delete_easy_logs_cache():
    dtu.get_cached('EasyLogsDB', lambda:None, just_delete=True)

    cache_dir = dtu.get_duckietown_cache_dir()
    fn = os.path.join(cache_dir, 'candidate_cloud.yaml')

    if os.path.exists(fn):
        dtu.logger.info('Removing %s' % fn)
        os.unlink(fn)


def get_easy_logs_db_cached_if_possible(write_candidate_cloud=False):
    return get_easy_logs_db_cached_if_possiblew(EasyLogsDB, write_candidate_cloud)


def get_easy_logs_db_cloud_cached_if_possible(write_candidate_cloud=False):
    return get_easy_logs_db_cached_if_possiblew(get_easy_logs_db_cloud, write_candidate_cloud)


def get_easy_logs_db_cached_if_possiblew(f, write_candidate_cloud):
    if EasyLogsDB._singleton is None:
        EasyLogsDB._singleton = dtu.get_cached('EasyLogsDB', f)

        cache_dir = dtu.get_duckietown_cache_dir()
        fn = os.path.join(cache_dir, 'candidate_cloud.yaml')

        if not os.path.exists(fn):
            logs = copy.deepcopy(EasyLogsDB._singleton.logs)
            # remove the field "filename"
            for k, v in logs.items():
                logs[k] = v._replace(filename=None)

            if write_candidate_cloud:
                s = yaml_representation_of_phy_logs(logs)
                dtu.write_data_to_file(s, fn)

            # try reading

                print('reading back logs')
                logs2 = logs_from_yaml(dtu.yaml_load_plain(s))
                print('read back %s' % len(logs2))

    return EasyLogsDB._singleton


def logs_from_yaml(data):
    dtu.check_isinstance(data, dict)

    res = OrderedDict()
    for k, d in data.items():
        res[k] = physical_log_from_yaml(d)
    return res


def yaml_representation_of_phy_logs(logs):
    dtu.check_isinstance(logs, dict)
    res = OrderedDict()
    for k, l in logs.items():
        res[k] = yaml_from_physical_log(l)
    s = yaml_dump_pretty(res)
    return s


def get_easy_logs_db_fresh():
    if EasyLogsDB._singleton is None:
        f = EasyLogsDB
        EasyLogsDB._singleton = f()
    return EasyLogsDB._singleton


def get_easy_logs_db_cloud():
    cloud_file = dtu.require_resource('cloud2.yaml')

    with dtu.timeit_wall("loading DB"):
        dtu.logger.info('Loading cloud DB %s' % dtu.friendly_path(cloud_file))
        data = dtu.yaml_load_file(cloud_file, plain_yaml=True)
        dtu.logger.debug('Conversion')
        logs = logs_from_yaml(data)

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
            aliases = OrderedDict()
            aliases.update(self.logs)
            # adding aliases unless we are asking for everything
            if query != '*':
                #print('adding more (query = %s)' % query)
                for _, log in self.logs.items():
                    dtr = DTR.from_yaml(log.resources['bag'])

                    original_name = dtr.name

                    # print ('alias: %s %s' % (original_name, dtr.name))
                    aliases[original_name] = log
                    original_name = original_name.replace('.bag', '')
                    aliases[original_name] = log

            result = dtu.fuzzy_match(query, aliases, filters=filters,
                                     raise_if_no_matches=raise_if_no_matches)
            # remove doubles after
            # XXX: this still has bugs
            present = defaultdict(set)
            for k, v in result.items():
                present[id(v)].add(k)

            def choose(options):
                if len(options) == 1:
                    return list(options)[0]
                else:
                    options = sorted(options, key=len)
                    return options[0]

            c = OrderedDict()
            for k, v in result.items():
                chosen = choose(present[id(v)])
                if k == chosen:
                    c[k] = v

            return c


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
    forbidden = [',', '(', 'conflicted'
                 #, ' '
                 ]
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
    patterns = [
        '*.bag',
        # We only care about <log>.XXX.mp4
        '*.*.mp4',
        '*.*.jpg',
        '*.*.webm',
        '*.*.png',
        '*.*.mov',
        '*.*.mts',
        '*.*.gif',
    ]
    basename2filename = dtu.look_everywhere_for_files(patterns=patterns, silent=True)
    base2basename2filename = defaultdict(lambda: {})
    for basename, fn in basename2filename.items():
        base = _get_base_base(basename)
#        print('basename: %s base: %s filename: %s' % (basename, base, fn))
        base2basename2filename[base][basename] = fn
    return AllResources(basename2filename=basename2filename,
                        base2basename2filename=base2basename2filename)


def load_all_logs():
    raise_if_duplicated = False
    all_resources = get_all_resources()

    logs = OrderedDict()
    ignored = []
    for basename, filename in all_resources.basename2filename.items():
        if not basename.endswith('.bag'):
            continue

        censor = ['ii-datasets', 'RCDP', '160122_3cars_dark-mercedes']
        to_censor = False
        for c in censor:
            if c in filename:
                to_censor = True
        if to_censor:
            ignored.append(filename)
            dtu.logger.warn('Ignoring %s' % filename)
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

        if l.log_name in logs:
            old = logs[l.log_name]

            old_sha1 = DTR.from_yaml(old.resources['bag']).hash['sha1']
            new_sha1 = DTR.from_yaml(l.resources['bag']).hash['sha1']

            if old_sha1 == new_sha1:
                # just a duplicate
                msg = 'File is a duplicate: %s ' % filename
                dtu.logger.warn(msg)
                continue
            # Actually a different log
            msg = 'Found twice this log: %s' % l.log_name
            msg += '\nProbably it is a processed version.'
            msg += "\n\nVersion 1:"

            msg += '\n\n' + dtu.indent(str(old), '  ')
            msg += "\n\n\nVersion 2:"
            msg += '\n\ncurrent: %s' % filename
            msg += '\n\ncurrent: %s' % ("RCDP" in filename)
            msg += '\n\n' + dtu.indent(str(l), '  ')
            if raise_if_duplicated:
                raise Exception(msg)
            else:
                dtu.logger.error(msg)

        logs[l.log_name] = l

    return logs

#
#def choose_hash_url_from_list(l):
#    for k in l:
#        if k.startswith('hash'):
#            return k
#    raise ValueError(l)


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
        for s, filename_resource in base2basename2filename[_base].items():
            basedot = _base + '.'
            if s.startswith(basedot):
                rest = s[len(basedot):]
                record_name = rest.lower()
                if not ignore_record(record_name):
                    dtr = create_dtr_version_1(filename_resource)
                    resources[record_name] = dtr

    # at least the bag file should be present
    assert 'bag' in resources

    return l


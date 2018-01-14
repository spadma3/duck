import os

import duckietown_utils as dtu
from duckietown_utils.cli import D8App
from duckietown_utils.test_hash import parse_hash_url
from easy_logs.logs_db import get_all_resources, delete_easy_logs_cache

from .logs_db import get_easy_logs_db_cached_if_possible, \
    get_easy_logs_db_cloud, get_easy_logs_db_fresh
from .logs_structure import PhysicalLog

__all__ = ['D8AppWithLogs']


class D8AppWithLogs(D8App):
    """
        An app that works with a log database.

        Adds the options --cache and --cloud, for working with logs.
    """

    def define_program_options(self, params):
        if hasattr(self, '_define_options_compmake'):
            self._define_options_compmake(params)
        self._define_my_options(params)
        self.define_options(params)

    def _define_my_options(self, params):
        g = "Options regarding the logs database"
        params.add_flag('cache', help="Use local log cache if it exists. Write cache if it does not.", group=g)
        params.add_flag('cache_reset', help="Delete the local log cache if it exists.", group=g)

        params.add_flag('cloud', help="Use cloud DB", group=g)
        self._db = None

    def get_easy_logs_db(self):
        if self._db is not None:
            return self._db

        use_cache = self.options.cache
        use_cloud = self.options.cloud
        if use_cache and use_cloud:
            msg = 'Cannot use --cache and --cloud together.'
            raise dtu.DTUserError(msg)

        if self.options.cache_reset:
            delete_easy_logs_cache()

        if use_cache:
            db = get_easy_logs_db_cached_if_possible()
        elif use_cloud:
            db = get_easy_logs_db_cloud()
        else:
            db = get_easy_logs_db_fresh()

        self._db = db
        return db


@dtu.contract(log=PhysicalLog, returns=PhysicalLog)
def download_if_necessary(log):
    """
        Downloads the log if necessary.

        Use like this:

            log = ...

            assert log.filename is None

            log2 = self.download_if_necessary(log.filename)

            open log2.filename

    """
    dtu.check_isinstance(log, PhysicalLog)
    # XXX: this is a bit convoluted
#    local_db = get_easy_logs_db_fresh()
#    logs = local_db.query(log.log_name, raise_if_no_matches=False)

    filename = get_log_if_not_exists(log)
    log2 = log._replace(filename=filename)
    return log2


@dtu.contract(log=PhysicalLog, returns=str)
def get_log_if_not_exists(log):
    """" Returns the path to the log. """
    downloads = dtu.get_duckietown_local_log_downloads()

    bag_url = log.resources['bag']
    print('bag url: %s' % bag_url)

    parsed = parse_hash_url(bag_url)
    print('parsed: %s' % str(parsed))

    all_resources = get_all_resources()
    if parsed.name in all_resources.basename2filename:
        # local!
        filename = all_resources.basename2filename[parsed.name]
        dtu.logger.info('We already have %s locally at %s' % (parsed.name, filename))
        return filename

#    if log_name.endswith('.bag'):
#        msg = 'get_log_if_not_exists() wants a log name, not a bag file'
#        dtu.logger.warn(msg)
#        log_name = log_name.replace('.bag', '')
#
#    if log_name in logs and (logs[log_name].filename is not None):
#        where = logs[log_name].filename
#        dtu.logger.info('We already have %s locally at %s' % (log_name, where))
#        return where
#    else:

    dtu.logger.info('We do not have %s locally.' % parsed.name)

    filename = os.path.join(downloads, parsed.name)
    if os.path.exists(filename):
        dtu.logger.info('It was already downloaded as %s' % filename)
        return filename

    dtu.require_resource_from_hash_url(bag_url, destination=filename)
    return filename


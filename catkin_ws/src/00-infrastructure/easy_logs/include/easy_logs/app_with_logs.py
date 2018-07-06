import os

import duckietown_utils as dtu
from duckietown_utils.cli import D8App
from duckietown_utils.download import download_if_not_exist
from easy_logs.logs_db import \
    get_easy_logs_db_cloud_cached_if_possible
from easy_logs.resource_desc import DTR

from .logs_db import get_all_resources, delete_easy_logs_cache, get_easy_logs_db_cached_if_possible, \
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
        params.add_flag('cache', help="(deprecated)", group=g)
        params.add_flag('fresh', help="Do not use cache.", group=g)

        params.add_flag('cache_reset', help="Delete the local log cache if it exists.", group=g)

        params.add_flag('cloud', help="Use cloud DB", group=g)

        params.add_flag('write_candidate_cloud', help="Prepare cloud DB", group=g)
        self._db = None

    def get_easy_logs_db(self):
        if self._db is not None:
            return self._db

        if self.options.cache:
            msg = 'Deprecated option --cache'
            dtu.logger.warning(msg)

        use_cache = not self.options.fresh
        use_cloud = self.options.cloud

        write_candidate_cloud = self.options.write_candidate_cloud

#        if use_cache and use_cloud:
#            msg = 'Cannot use --cache and --cloud together.'
#            raise dtu.DTUserError(msg)

        if self.options.cache_reset:
            delete_easy_logs_cache()

        if use_cloud:
            if use_cache:
                db = get_easy_logs_db_cloud_cached_if_possible(write_candidate_cloud)
            else:
                db = get_easy_logs_db_cloud()
        else:
            if use_cache:
                db = get_easy_logs_db_cached_if_possible(write_candidate_cloud)
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

    filename = get_log_if_not_exists(log)
    log2 = log._replace(filename=filename)
    return log2


@dtu.contract(log=PhysicalLog, returns=str)
def get_log_if_not_exists(log):
    """" Returns the path to the log. """
    downloads = dtu.get_duckietown_local_log_downloads()

    dtr_yaml = log.resources['bag']
    dtr = DTR.from_yaml(dtr_yaml)
#    print dtr_yaml
#    print('bag url: %s' % bag_url)

#    parsed = dtu.parse_hash_url(bag_url)
#    print('parsed: %s' % str(parsed))

    all_resources = get_all_resources()
    if dtr.name in all_resources.basename2filename:
        # local!
        filename = all_resources.basename2filename[dtr.name]
        dtu.logger.info('We already have %s locally at %s' % (dtr.name, filename))
        return filename

    dtu.logger.info('We do not have %s locally.' % dtr.name)

    filename = os.path.join(downloads, dtr.name)
    if os.path.exists(filename):
        dtu.logger.info('It was already downloaded as %s' % filename)
        return filename

    use = []
    for url in dtr.urls:
        if url.startswith('http'):
            use.append(url)

    def priority(x):
        if '8080' in x:
            return 0
        else:
            return 1

    use.sort(key=priority)

    dtu.logger.info('URLS: \n' + '\n'.join(use))

    for url in use:
        try:
            print('Trying %s' % url)
            dtu.d8n_make_sure_dir_exists(filename)
            download_if_not_exist(url, filename)
        except Exception as e:  # XXX
            dtu.logger.error(e)
        else:
            break
    else:
        log.error('could not download any file')

    #        parsed = parse_hash_url(hash_url)
    #        basename = parsed.name if parsed.name is not None else parsed.sha1
    #        dirname = get_duckietown_cache_dir()
    #        destination = os.path.join(dirname, basename)
    #
    #    d8n_make_sure_dir_exists(destination)
    #    download_if_not_exist(url, destination)
    #    return destination

#    dtu.require_resource_from_hash_url(bag_url, destination=filename)
    return filename


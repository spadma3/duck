import duckietown_utils as dtu

from .logs_db import get_easy_logs_db_cached_if_possible, \
    get_easy_logs_db_cloud, get_easy_logs_db_fresh
from .logs_structure import PhysicalLog
from .mis import get_log_if_not_exists

__all__ = ['D8AppWithLogs']


class D8AppWithLogs(dtu.D8App):
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
        params.add_flag('cache', help="Use local log cache.", group=g)
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

        if use_cache:
            db = get_easy_logs_db_cached_if_possible()
        elif use_cloud:
            db = get_easy_logs_db_cloud()
        else:
            db = get_easy_logs_db_fresh()

        self._db = db
        return db

    @dtu.contract(log=PhysicalLog, returns=PhysicalLog)
    def download_if_necessary(self, log):
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
        local_db = get_easy_logs_db_fresh()
        logs = local_db.query(log.log_name, raise_if_no_matches=False)
        filename = get_log_if_not_exists(logs, log.log_name)
        log2 = log._replace(filename=filename)
        return log2

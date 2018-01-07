import duckietown_utils as dtu
from easy_logs.app_with_logs import D8AppWithLogs, download_if_necessary
from easy_logs.logs_db import get_easy_logs_db_cloud
from quickapp.quick_app import QuickApp

from ..logs_db import get_easy_logs_db_fresh

__all__ = [
    'Download',
]


class Download(D8AppWithLogs, QuickApp):
    """
        Downloads logs if necessary..
    """

    cmd = 'dt-easy_logs-download'

    usage = """

Usage:

    $ %(prog)s  "log query"


"""

    def define_options(self, params):
        params.accept_extra()

    def define_jobs_context(self, context):
        # look into cloud db
        extra = self.options.get_extra()
        if not extra:
            msg = 'Please specify a log.'
            raise dtu.DTUserError(msg)
        else:
            query = extra

        db_cloud = get_easy_logs_db_cloud()
        logs = db_cloud.query(query)

        db_local = get_easy_logs_db_fresh()

        n = 0

        logs_to_download = {}
        for log in logs.values():
            physical_log_name = log.log_name

            if physical_log_name in logs_to_download:
                continue

            local = db_local.query(physical_log_name, raise_if_no_matches=False)
            if local:
                msg = 'I have already %s' % physical_log_name
                self.info(msg)
            else:
                l = context.comp(download_if_necessary, log)
                logs_to_download[physical_log_name] = l
                n += 1
                msg = 'I will get     %s' % physical_log_name
                self.info(msg)

        context.comp(done, logs_to_download)


def done(logs_to_download):
    msg = 'Done downloading %s logs.' % len(logs_to_download)
    dtu.logger.info(msg)


def download_job(log):
    download_if_necessary(log)


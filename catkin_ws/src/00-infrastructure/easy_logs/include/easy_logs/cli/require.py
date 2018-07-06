from decent_params.utils.script_utils import UserError
import duckietown_utils as dtu
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTNoMatches
from easy_logs.app_with_logs import D8AppWithLogs, download_if_necessary
from easy_logs.logs_db import get_easy_logs_db_cloud, \
    get_easy_logs_db_cloud_cached_if_possible
from quickapp import QuickApp, QuickAppBase

from ..logs_db import get_easy_logs_db_fresh

__all__ = [
    'Download',
]


class Download(D8AppWithLogs, QuickAppBase):
    """
        Downloads logs if necessary..
    """

    cmd = 'dt-easy_logs-download'

    usage = """

Usage:

    $ %(prog)s  "log query"


"""

    def define_program_options(self, params):
        self._define_my_options(params)
        params.accept_extra()

    def go(self):
        # look into cloud db
        extra = self.options.get_extra()
        if not extra:
            msg = 'Please specify a log.'
            raise dtu.DTUserError(msg)
        else:
            query = extra

        if self.options.fresh:
            db_cloud = get_easy_logs_db_cloud()
        else:
            db_cloud = get_easy_logs_db_cloud_cached_if_possible()

        try:
            logs = db_cloud.query(query)
        except DTNoMatches as e:
            msg = 'Could not find the logs matching the query.'
            raise_wrapped(UserError, e, msg, compact=True)

        db_local = get_easy_logs_db_fresh()

        n = 0

        logs_to_download = {}
        # noinspection PyUnboundLocalVariable
        for log in logs.values():
            physical_log_name = log.log_name

            if physical_log_name in logs_to_download:
                continue

            local = db_local.query(physical_log_name, raise_if_no_matches=False)
            if local and local[physical_log_name].filename is not None:

                msg = 'I have already %s' % physical_log_name
                self.info(msg)
                self.info(local.filename)
            else:
                # job_id = 'download-%s' % physical_log_name
                # l = context.comp(download_if_necessary, log, job_id=job_id)
                l = download_if_necessary(log)
                logs_to_download[physical_log_name] = l
                n += 1
                # msg = 'I will get %s' % physical_log_name
                # self.info(msg)

        # msg = 'Done downloading %s logs.' % len(logs_to_download)
#
#         context.comp(done, logs_to_download)
#
#
# def done(logs_to_download):
#     msg = 'Done downloading %s logs.' % len(logs_to_download)
#     dtu.logger.info(msg)
#
#
# def download_job(log):
#     download_if_necessary(log)


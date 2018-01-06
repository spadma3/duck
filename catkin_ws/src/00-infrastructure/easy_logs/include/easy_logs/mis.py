import os
import duckietown_utils as dtu


def get_log_if_not_exists(logs, log_name):
    """" Returns the path to the log. """
    downloads = dtu.get_duckietown_local_log_downloads()

    if log_name.endswith('.bag'):
    #    msg = 'get_log_if_not_exists() wants a log name, not a bag file'
    #    raise ValueError(msg)
        log_name = log_name.replace('.bag', '')

    if log_name in logs and (logs[log_name].filename is not None):
        where = logs[log_name].filename
        dtu.logger.info('We already have %s locally at %s' % (log_name, where))
        return where
    else:
        dtu.logger.info('We do not have %s locally.' % log_name)

        filename = os.path.join(downloads, log_name + '.bag')
        if os.path.exists(filename):
            dtu.logger.info('It was already downloaded as %s' % filename)
            return filename

        resource = log_name + '.bag'
        dtu.require_resource(resource, destination=filename)
        return filename


import os

from duckietown_utils import get_duckietown_local_log_downloads
from duckietown_utils import logger
from duckietown_utils import require_resource
from easy_logs.logs_db import get_easy_logs_db_fresh


def require_main(log_names='*'):
    required = []
    for name in log_names:
        if not name.endswith('.bag'):
            name = name.replace('.bag','')
        required.append(name)
    
    db = get_easy_logs_db_fresh()

    for name in required:
        get_log_if_not_exists(db.logs, name)

def get_log_if_not_exists(logs, log_name):
    """" Returns the path to the log. """
    downloads = get_duckietown_local_log_downloads()
    
    
    if log_name.endswith('.bag'):
        msg = 'get_log_if_not_exists() wants a log name, not a bag file'
        raise ValueError(msg)
    
    if log_name in logs and (logs[log_name].filename is not None):
        where = logs[log_name].filename 
        logger.info('We already have %s locally at %s' % (log_name, where))
        return where
    else:
        logger.info('We do not have %s locally.' % log_name)
        
        
        filename = os.path.join(downloads, log_name + '.bag')
        if os.path.exists(filename):
            logger.info('It was already downloaded as %s' % filename)
            return filename
        
        resource = log_name + '.bag'
        require_resource(resource, destination=filename)
        return filename
    
        
    
    
    
    
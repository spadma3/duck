import os

import yaml

from duckietown_utils import logger
from duckietown_utils.constants import get_duckietown_local_log_downloads
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.system_cmd_imp import system_cmd_result
from easy_logs.logs_db import get_urls_path,\
    get_easy_logs_db_fresh
from duckietown_utils.download import download_if_not_exist

def get_dropbox_urls():
    f = get_urls_path()
    if not os.path.exists(f):
        raise DTConfigException(f)
    data = open(f).read()
    
    urls = yaml.load(data)
    
    def sanitize(url):
        if url.endswith('?dl=0'):
            url = url.replace('?dl=0','?dl=1')
        return url
    
    return dict([(k, sanitize(url)) for k, url in urls.items()])

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
    urls = get_dropbox_urls()
    
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
        
#         bag_name = log_name + '.bag'
        if not log_name in urls:
            msg = 'No URL found for %r.' % log_name
            raise Exception(msg)
        else:
            url = urls[log_name]
            
            download_if_not_exist(url, filename)
        return filename
    
    
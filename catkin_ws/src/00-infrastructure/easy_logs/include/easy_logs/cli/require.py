import os

import yaml

from duckietown_utils import logger
from duckietown_utils.constants import get_duckietown_local_log_downloads
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.system_cmd_imp import system_cmd_result
from easy_logs.logs_db import get_easy_logs_db, get_urls_path

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
        
    urls = get_dropbox_urls()
    downloads = get_duckietown_local_log_downloads()
    
    db = get_easy_logs_db()
    logs = db.logs
    
    for name in required:
        if name in logs:
            logger.info('We already have %s locally at %s' % (name, logs[name].filename))
            #continue
        else:
            logger.info('We do not have %s locally.' % name)
        filename = os.path.join(downloads, name + '.bag')
        if os.path.exists(filename):
            logger.info('It was already downloaded as %s' % filename)
            continue
    
        if not name in urls:
            msg = 'No url found for %r.' % name
            raise Exception(msg)
        else:
            url = urls[name]
            
            
            if not os.path.exists(filename):
                download_url_to_file(url, filename)
                
def download_url_to_file(url, filename):
    logger.info('Download from %s' % (url))
    tmp = '/tmp/download'
    cmd = [
        'wget',
        '-O',
        tmp,
        url
    ]
    _ = system_cmd_result(cwd='.', cmd=cmd,
              display_stdout=False,
              display_stderr=False,
              raise_on_error=True,
              write_stdin='',
              capture_keyboard_interrupt=False,
              env=None)
    os.rename(tmp, filename)
                
    logger.info('-> %s' % filename)
                
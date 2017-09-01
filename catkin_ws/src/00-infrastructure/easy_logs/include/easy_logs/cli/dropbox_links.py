import os

from ruamel import yaml

from duckietown_utils import logger
from duckietown_utils.download import get_urls_path
from duckietown_utils.system_cmd_imp import system_cmd_result
from easy_logs.logs_db import get_easy_logs_db


def dropbox_links_main(query):
    logger.info('NOTE: Run this inside ~/Dropbox/duckietown-data. ')
    logger.info('NOTE: There are some hard-coded paths to modify in dropbox_links.py')
    
    output = get_urls_path()
    if os.path.exists(output):
        urls = yaml.safe_load(open(output).read())
        for k,v in list(urls.items()):
            if not v.startswith('http'):
                del urls[k]
    else:
        urls = {}
    command = '/home/andrea/bin/dropbox'
    base = '/mnt/dorothy-duckietown-data/'
    db = get_easy_logs_db()
    logs  = db.query(query)
    logger.info('Found %d logs.' % len(logs))
    
    for logname, log in logs.items():
        if logname in urls:
            logger.info('Already have %s' % logname)
            continue
        
        filename = log.filename
        only = filename.replace(base, '')

        cmd = [command, 'sharelink', only]
        res = system_cmd_result(cwd='.', cmd=cmd,
                      display_stdout=False,
                      display_stderr=True,
                      raise_on_error=True,
                      write_stdin='',
                      capture_keyboard_interrupt=False,
                      env=None)
        link = res.stdout.strip()
        if 'responding' in link:
            logger.debug('Dropbox is not responding, I will stop here.')
            
            break
        
        logger.info('link : %s' % link)
        urls[logname] = link
    
    yaml.default_flow_style = True
#     yaml.indent(mapping=2, sequence=4, offset=2)
    with open(output, 'w') as f:
        yaml.dump(urls, f)
        

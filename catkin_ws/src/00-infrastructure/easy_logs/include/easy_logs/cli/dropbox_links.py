import os

from ruamel import yaml

import duckietown_utils as dtu

from easy_logs import get_easy_logs_db


def dropbox_links_main(query):
    dtu.logger.info('NOTE: Run this inside ~/Dropbox/duckietown-data. ')
    dtu.logger.info('NOTE: There are some hard-coded paths to modify in dropbox_links.py')
    
    output = dtu.get_urls_path()
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
    dtu.logger.info('Found %d logs.' % len(logs))
    
    for logname, log in logs.items():
        if logname in urls:
            dtu.logger.info('Already have %s' % logname)
            continue
        
        filename = log.filename
        only = filename.replace(base, '')

        cmd = [command, 'sharelink', only]
        res = dtu.system_cmd_result(cwd='.', cmd=cmd,
                      display_stdout=False,
                      display_stderr=True,
                      raise_on_error=True,
                      write_stdin='',
                      capture_keyboard_interrupt=False,
                      env=None)
        link = res.stdout.strip()
        if 'responding' in link:
            dtu.logger.debug('Dropbox is not responding, I will stop here.')
            
            break
        
        dtu.logger.info('link : %s' % link)
        urls[logname] = link
    
    yaml.default_flow_style = True
#     yaml.indent(mapping=2, sequence=4, offset=2)
    with open(output, 'w') as f:
        yaml.dump(urls, f)
        

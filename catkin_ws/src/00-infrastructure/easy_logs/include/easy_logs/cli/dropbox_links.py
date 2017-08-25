from duckietown_utils import logger
from easy_logs.logs_db import get_easy_logs_db
from duckietown_utils.system_cmd_imp import system_cmd_result


def dropbox_links_main(query):
    command = '/home/andrea/bin/dropbox'
    base = '/mnt/dorothy-duckietown-data/'
    db = get_easy_logs_db()
    logs  = db.query(query)
    logger.info('Found %d logs.' % len(logs))
    for log in logs.values():
        
        filename = log.filename
        only = filename.replace(base, '')

        cmd = [command, 'sharelink', only]
        res = system_cmd_result(cwd='.', cmd=cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=True,
                      write_stdin='',
                      capture_keyboard_interrupt=False,
                      env=None)
        link = res.stdout.strip()
        logger.info('link : %s' % link)
    
import os
from duckietown_utils import logger
from duckietown_utils.system_cmd_imp import system_cmd_result

def download_if_not_exist(url, filename):
    if not os.path.exists(filename):
        download_url_to_file(url, filename)
    return filename

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
                
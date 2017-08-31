import os

from duckietown_utils import logger

from .friendly_path_imp import friendly_path
from .system_cmd_imp import system_cmd_result
from .constants import get_duckietown_root
from .test_hash import get_md5
from .mkdirs import d8n_make_sure_dir_exists



def download_if_not_exist(url, filename):
    if not os.path.exists(filename):
        logger.info('Path does not exist: %s'% filename)
        download_url_to_file(url, filename)
    return filename

def download_url_to_file(url, filename):
    logger.info('Download from %s' % (url))
    tmp = filename + '.download'
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
    
    d8n_make_sure_dir_exists(filename)
    os.rename(tmp, filename)
                
    logger.info('-> %s' % friendly_path(filename))

                
def get_file_from_url(url):
    """ 
        Returns a local filename corresponding to the contents of the URL.
        The data is cached in caches/downloads/
    """
    basename = get_md5(url)
    if 'jpg' in url:
        basename += '.jpg'
    filename = os.path.join(get_duckietown_root(), 'caches', 'downloads', basename)
    download_if_not_exist(url, filename)
    return filename

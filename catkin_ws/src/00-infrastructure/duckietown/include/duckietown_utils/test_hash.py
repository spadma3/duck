import hashlib
import os
import urllib
import urlparse

from duckietown_utils.logging_logger import logger

from .caching import get_cached
from .timeit import timeit_wall

__all__ = [
    'get_md5',
    'sha1_for_file',
    'create_hash_url',
]


def get_md5(contents):
    m = hashlib.md5()
    m.update(contents)
    s = m.hexdigest()
    return s


def sha1_for_file(path, block_size=256 * 128):
    '''
    Block size directly depends on the block size of your filesystem
    to avoid performances issues.
    '''
    logger.debug('sha1 for %s' % path)
    h = hashlib.sha1()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(block_size), b''):
            h.update(chunk)
    return h.hexdigest()


def sha1_for_file_cached(filename):

    def f():
        return sha1_for_file(filename)

    basename = os.path.basename(filename)
    cache_name = 'sha1_for_file/' + basename
    return get_cached(cache_name, f, quiet=True)


def create_hash_url(fn):
    # scheme://netloc/path;parameters?query#fragment
    scheme = 'hash'
    netloc = 'sha1'
    with timeit_wall("hashing %s" % fn, minimum=500):
        path = sha1_for_file_cached(fn)
    parameters = None
    name = os.path.basename(fn)
    size = os.path.getsize(fn)
    qs = [ ('size', size), ('name', name) ]
    query = urllib.urlencode(qs)
    fragment = None

    url = urlparse.urlunparse((scheme, netloc, path, parameters, query, fragment))
    return url


from .path_utils import expand_all

from duckietown_utils import logger
import os 
from .friendly_path_imp import friendly_path

__all__ = [
    'get_cached',
]

def get_cached(cache_name, f, quiet='not-given'):
    """ 
        Caches the result of f() in a file called
            ${DUCKIETOWN_ROOT}/caches/![name].cache.pickle
    """
            
    import cPickle
    
    cache = '${DUCKIETOWN_ROOT}/caches/%s.cache.pickle' % cache_name
    cache = expand_all(cache)
    
    if quiet == 'not-given':
        should_be_quiet = False
    else: 
        should_be_quiet = quiet
        
    if os.path.exists(cache):
        
        if not should_be_quiet:
            logger.info('Using cache %s' % friendly_path(cache))
        with open(cache) as f:
            ob = cPickle.load(f)
    else:
        ob = f()
        if not should_be_quiet:
            logger.info('Writing to cache %s' % friendly_path(cache))
        try:
            os.makedirs(os.path.dirname(cache))
        except:
            pass
        with open(cache, 'w') as f:
            cPickle.dump(ob, f)
#         
#                 
    return ob

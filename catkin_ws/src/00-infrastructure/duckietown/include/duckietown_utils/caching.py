from duckietown_utils.path_utils import expand_all, display_filename
import cPickle
from duckietown_utils import logger
import os
from ruamel import yaml
import copy



def get_cached(cache_name, f, quiet='not-given'):
    """ 
        Caches the result of f() in a file called
            ${DUCKIETOWN_ROOT}/caches/![name].cache.pickle
    """
            
    cache = '${DUCKIETOWN_ROOT}/caches/%s.cache.pickle' % cache_name
    cache = expand_all(cache)
    
    if quiet == 'not-given':
        should_be_quiet = False
    else: 
        should_be_quiet = quiet
        
    if os.path.exists(cache):
        
        if not should_be_quiet:
            logger.info('Using cache %s' % display_filename(cache))
        with open(cache) as f:
            ob = cPickle.load(f)
    else:
        ob = f()
        if not should_be_quiet:
            logger.info('Writing to cache %s' % display_filename(cache))
        try:
            os.makedirs(os.path.dirname(cache))
        except:
            pass
        with open(cache, 'w') as f:
            cPickle.dump(ob, f)
        
        logs = copy.deepcopy(ob.logs)
        for k, v in logs.items():
            logs[k]=v._replace(filename= None)
        with open(cache+'.yaml', 'w') as f:
            logs_yaml = yaml.dump(logs)
            f.write(logs_yaml)
                
    return ob

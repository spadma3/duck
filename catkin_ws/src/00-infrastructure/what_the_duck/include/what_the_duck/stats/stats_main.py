from what_the_duck.mongo_suppor import get_upload_collection
from collections import defaultdict
from duckietown_utils import logger

def get_valid_data(collection):
    
    res = list(collection.find())
    out = []
    for r in res:
        if 'what_the_duck_version' in r:
            if r['what_the_duck_version'] in ['1.1']:
                out.append(r)
    
    logger.info('Found %d database entries; %d are compatible.' % 
                (len(res), len(out)))
    return out
    
    
def what_the_duck_stats():
    collection = get_upload_collection()
    
    res = list( get_valid_data(collection))
    
    hostnames = defaultdict(lambda:0)
    
    for r in res:
        hostnames[r['hostname']] += 1

        
    s = 'Number of tests by hostname:\n '
    for h, n in hostnames.items():
        s += '- %s: %d tests' % (h, n)
    
    
    logger.info(s)
    
    
# -*- coding: utf-8 -*-
from what_the_duck.mongo_suppor import get_upload_collection
from collections import defaultdict
from duckietown_utils import logger, write_data_to_file
from what_the_duck.stats.output import create_summary
import sys
from duckietown_utils.safe_pickling import safe_pickle_dump

def get_valid_data(collection):
    
    res = list(collection.find())
    out = []
    for r in res:
        if 'what_the_duck_version' in r:
            v = r['what_the_duck_version']
            if v in ['1.1', '1.2', '1.3']:
                del r['_id']
                
                if v in ['1.1']:
                    r['type'] = '?'
    
                out.append(r)
                
    logger.info('Found %d database entries; %d are compatible.' % 
                (len(res), len(out)))
    return out
    
    
def what_the_duck_stats():
    
    if len(sys.argv) > 1:
        output =  sys.argv[1]
    else:
        output = 'what_the_duck_stats.html'
        
    collection = get_upload_collection()
    
    res = list(get_valid_data(collection))


#     logger.debug('dumping last YAML')
#     import yaml
#     data = yaml.dump(res)
#     write_data_to_file(data, 'last_download.yaml')
    logger.debug('dumping Pickle')
    safe_pickle_dump(res, 'last_download.pickle')
        
    hostnames = defaultdict(lambda:0)
    
    for r in res:
        hostnames[r['hostname']] += 1

    s = 'Number of tests by hostname: '
    for h, n in hostnames.items():
        s += '\n- %s: %d tests' % (h, n)
    
    logger.info(s)
    
    html = create_summary(res)
    write_data_to_file(html, output)
    
    
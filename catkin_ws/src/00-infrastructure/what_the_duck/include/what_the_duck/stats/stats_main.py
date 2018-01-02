# -*- coding: utf-8 -*-
from what_the_duck.mongo_suppor import get_upload_collection
from collections import defaultdict
import duckietown_utils as dtu

from what_the_duck.stats.output import create_summary
import sys

def get_valid_data(collection):
    dtu.logger.info('downloading database')
    res = []
    for i, x in  enumerate(collection.find()):
        if i != 100 == 0:
            dtu.logger.debug('Downloaded %s' % i)
        res.append(x)
#     res = list(collection.find())
    dtu.logger.info('downloaded %s' % len(res))
    out = []
    for r in res:
        if 'what_the_duck_version' in r:
            v = r['what_the_duck_version']
            if v in ['1.1', '1.2', '1.3', '1.4']:
                del r['_id']
                
                if v in ['1.1']:
                    r['type'] = '?'
    
                out.append(r)
                
    dtu.logger.info('Found %d database entries; %d are compatible.' % 
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
    dtu.logger.debug('dumping Pickle')
    dtu.safe_pickle_dump(res, 'last_download.pickle')
        
    hostnames = defaultdict(lambda:0)
    
    for r in res:
        hostnames[r['hostname']] += 1

    s = 'Number of tests by hostname: '
    for h, n in hostnames.items():
        s += '\n- %s: %d tests' % (h, n)
    
    dtu.logger.info(s)
    
    html = create_summary(res)
    dtu.write_data_to_file(html, output)
    
    
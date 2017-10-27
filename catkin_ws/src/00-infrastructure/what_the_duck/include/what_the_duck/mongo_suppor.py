from contracts import contract
import datetime
import getpass
import shelve
import socket

from duckietown_utils import is_internet_connected, logger, raise_wrapped
from duckietown_utils import on_duckiebot, on_laptop, on_circle
from what_the_duck import what_the_duck_version

from .constant import Result
from .geolocation import get_geolocation_data


mongo_db = 'wtd01'
mongo_collection = 'uploads'


try:
    import pymongo  # @UnresolvedImport @UnusedImport
except ImportError as e:
    msg = 'pymongo not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     pip install --user pymongo'
    raise_wrapped(Exception, e, msg)

def get_connection_string():    
    username = 'upload'
    password = 'quackquack'
    
    s = ("mongodb://<USERNAME>:<PASSWORD>@dt0-shard-00-00-npkyt.mongodb.net:27017,"
        "dt0-shard-00-01-npkyt.mongodb.net:27017,dt0-shard-00-02-npkyt.mongodb.net"
        ":27017/test?ssl=true&replicaSet=dt0-shard-0&authSource=admin")
    
    s = s.replace("<PASSWORD>", password)
    s = s.replace("<USERNAME>", username)
    return s

def get_local_keys():
    username = getpass.getuser()
    hostname = socket.gethostname()
    d = {}

    if on_duckiebot():
        stype = 'duckiebot'
    elif on_laptop():
        stype = 'laptop'
    elif on_circle():
        stype = 'cloud'
    else:
        stype = 'unknown'
        
    d['type'] = stype
    d['what_the_duck_version'] = what_the_duck_version
    d['username'] = username
    d['hostname'] = hostname
    now = datetime.datetime.now()
    date_s = now.isoformat('_')
    upload_event_id = hostname + '-' + date_s 
    d['upload_event_id'] = upload_event_id
    d['upload_event_date'] = now
    location = get_geolocation_data()
    d.update(location)
    return d
    
@contract(result=Result)
def json_from_result(result):
    d = {}
    d['test_name'] = result.entry.get_test_id()
    d['status'] = result.status
    d['out_short'] = result.out_short
    d['out_long'] = result.out_long
    return d
     
def get_upload_collection():
    s = get_connection_string()
    
    logger.info('Opening connection to MongoDB...')
    client = pymongo.MongoClient(s)
    
    db = client[mongo_db]
    collection = db[mongo_collection]
    return collection
    
def upload_results(results):
    to_upload = json_from_results(results)
    
    upload(to_upload)
    
def upload(to_upload):
    filename = '/tmp/what_the_duck'
    S = shelve.open(filename)
    try:
#         logger.debug('New records: %s  previous: %s' % (len(to_upload), len(S)))
        
        for u in to_upload:
            S[u['_id']] = u
        
        if not is_internet_connected():
            msg = 'Internet is not connected: cannot upload results.'
            logger.warning(msg)
        else:
            remaining = []
            for k in S:
                remaining.append(S[k])
                
            collection = get_upload_collection()
            logger.info('Uploading %s test results' % len(remaining))
            collection.insert_many(remaining)
            logger.info('done')
            for r in remaining:
                del S[r['_id']]
    finally:
#         logger.info('Remaining %s' % (len(S)))
        S.close()

        

def json_from_results(results):
    to_upload = []
    d0 = get_local_keys()
    for i, result in enumerate(results):
        d1 = json_from_result(result)
        d1['_id'] = d0['upload_event_id'] + '-%d' % i
        d1.update(d0)  
        to_upload.append(d1)
    return to_upload
    
    
    
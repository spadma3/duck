import datetime
from duckietown_utils import logger, raise_wrapped
import getpass
import socket

from contracts import contract
from what_the_duck import what_the_duck_version
from .constant import Result


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
    d['what_the_duck_version'] = what_the_duck_version
    d['username'] = username
    d['hostname'] = hostname
    now = datetime.datetime.now()
    date_s = now.isoformat('_')
    upload_event_id = hostname + '-' + date_s 
    d['upload_event_id'] = upload_event_id
    d['upload_event_date'] = now
    return d
    
@contract(result=Result)
def json_from_result(result):
    d = {}
    d['test_name'] = result.entry.get_test_id()
    d['status'] = result.status
    d['out_short'] = result.out_short
    d['out_long'] = result.out_short
    return d
#     Result = namedtuple('Result', 'entry status out_short out_long')
    
def upload_results(results):
    to_upload = []
    d0 = get_local_keys()
    
    for i, result in enumerate(results):
        d1 = json_from_result(result)
        d1.update(d0)  
        to_upload.append(d1)
    
    s = get_connection_string()
    
    logger.info('Opening DB connections')
    client = pymongo.MongoClient(s)
    
    db = client[mongo_db]
    collection = db[mongo_collection]
    logger.info('Inserting %s tests' % len(to_upload))
    collection.insert_many(to_upload)
    
    logger.info('done')

    
    
    
    
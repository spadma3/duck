from easy_logs.logs_db import get_easy_logs_db
from duckietown_utils.friendly_path_imp import friendly_path
from duckietown_utils import logger


def find_logs_main(query='*'):
    db = get_easy_logs_db()
    logs  = db.query(query)
    logger.info('Found %d logs.' % len(logs))
    for log in logs.values():
        print(friendly_path(log.filename))
    
    
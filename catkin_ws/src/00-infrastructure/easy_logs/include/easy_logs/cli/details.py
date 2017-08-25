from easy_logs.logs_db import get_easy_logs_db
from duckietown_utils import logger
import yaml


def details_main(query='*'):
    db = get_easy_logs_db()
    logs  = db.query(query)
    logger.info('Found %d logs.' % len(logs))
    for log in logs.values():
        
        s = yaml.dump(log._asdict())
        print(s)

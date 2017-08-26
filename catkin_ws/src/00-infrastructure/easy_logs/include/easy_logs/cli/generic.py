from abc import abstractmethod

from duckietown_utils.cli import D8App
from duckietown_utils.exceptions import DTUserError
from easy_logs.logs_db import get_easy_logs_db_cached_if_possible, get_easy_logs_db_cloud,\
    get_easy_logs_db_fresh


class RunLineDetectionTests(D8App): 
    """ Runs the line detection tests programmatically. """

    def define_options(self, params):
        params.add_flag('cache', help="Use cache") 
        params.add_flag('cloud', help="Use cloud cache")
        params.accept_extra()
        
        
    @abstractmethod
    def show_info(self, logs):
        pass
    
    
    def go(self):
        extra = self.options.get_extra()
        if not extra:
            query = '*'
        else:
            if len(extra) > 1:
                msg = 'Expected only one extra argument.'
                raise DTUserError(msg)
            
        use_cache = self.options.cache 
        use_cloud = self.options.cloud
        if use_cache and use_cloud:
            msg = 'Cannot use --cache and --cloud together.'
            raise DTUserError(msg)
        
        if use_cache:
            db = get_easy_logs_db_cached_if_possible()
        elif use_cloud:
            db = get_easy_logs_db_cloud()
        else:
            db = get_easy_logs_db_fresh()
        
        logs  = db.query(query)
    
        self.info('Found %d logs.' % len(logs))
        self.show_info(logs)

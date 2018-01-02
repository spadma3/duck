import duckietown_utils as dtu
from easy_logs.cli.easy_logs_summary_imp import easy_logs_summary
from easy_logs.logs_db import get_easy_logs_db_cloud

@dtu.unit_test
def call_summary():
    try:
        print(easy_logs_summary())
    except dtu.DTNoMatches:
        pass

@dtu.unit_test
def cloud():
    get_easy_logs_db_cloud()    
    
if __name__ == '__main__':
    dtu.run_tests_for_this_module()

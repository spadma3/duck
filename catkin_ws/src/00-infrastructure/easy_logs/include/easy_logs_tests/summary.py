from comptests.registrar import run_module_tests, comptest
from easy_logs.cli.easy_logs_summary_imp import easy_logs_summary
from duckietown_utils.exceptions import DTNoMatches
from easy_logs.logs_db import get_easy_logs_db_cloud

@comptest
def call_summary():
    try:
        print(easy_logs_summary())
    except DTNoMatches:
        pass

@comptest
def cloud():
    get_easy_logs_db_cloud()    
    
if __name__ == '__main__':
    run_module_tests()

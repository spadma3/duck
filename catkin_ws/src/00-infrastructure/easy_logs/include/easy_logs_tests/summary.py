from comptests.registrar import run_module_tests, comptest
from easy_logs.cli.easy_logs_summary_imp import easy_logs_summary
from duckietown_utils.exceptions import DTNoMatches

@comptest
def call_summary():
    try:
        print(easy_logs_summary())
    except DTNoMatches:
        pass
    
    
if __name__ == '__main__':
    run_module_tests()

from comptests.registrar import run_module_tests, comptest
from easy_logs.scripts.easy_logs_summary_imp import easy_logs_summary

@comptest
def call_summary():
    print(easy_logs_summary())
    
    
if __name__ == '__main__':
    run_module_tests()

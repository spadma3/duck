from easy_algo.scripts.summary_imp import summary
from comptests.registrar import comptest, run_module_tests


@comptest
def call_summary():
    summary()
    
    
if __name__ == '__main__':
    run_module_tests()

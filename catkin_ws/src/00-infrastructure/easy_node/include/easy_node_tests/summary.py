
from comptests.registrar import comptest, run_module_tests

from easy_node.user_config.summary import user_config_summary


@comptest
def call_summary(): 
    print(user_config_summary())
    
    
if __name__ == '__main__':
    run_module_tests()

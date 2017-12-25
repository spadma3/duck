from . import logger

__all__ = [
    'unit_test', 
    'run_tests_for_this_module',
]


try: 
    from comptests import comptest as unit_test  # @UnusedImport
    from comptests import run_module_tests as run_tests_for_this_module  # @UnusedImport
    logger.warning('Using the Comptests testing framework.')
    using_fake_tests = False
except ImportError:
    logger.warning('Unit tests are disabled becaused Comptests not found.')
    using_fake_tests = True
    
if using_fake_tests:
    def unit_test(f):
        return f
    
    def run_tests_for_this_module():
        pass
from . import logger
from .constants import DuckietownConstants

__all__ = [
    'unit_test', 
    'run_tests_for_this_module',
]

show_info = DuckietownConstants.debug_show_package_import_info

try: 
    from comptests import comptest as unit_test  # @UnusedImport
    from comptests import run_module_tests as run_tests_for_this_module  # @UnusedImport
    
    if show_info:
        logger.warning('Using the Comptests testing framework.')
    using_fake_tests = False
except ImportError:
    if show_info:
        logger.warning('Unit tests are disabled becaused Comptests not found.')
    using_fake_tests = True
    
if using_fake_tests:
    def unit_test(f):
        return f
    
    def run_tests_for_this_module():
        pass
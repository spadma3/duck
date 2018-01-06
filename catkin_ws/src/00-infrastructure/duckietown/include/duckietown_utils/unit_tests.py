from .constants import DuckietownConstants
from .logging_logger import logger

__all__ = [
    'unit_test',
    'run_tests_for_this_module',
    'get_output_dir_for_test',
]

show_info = DuckietownConstants.debug_show_package_import_info


def unit_test(f):
    return f


def run_tests_for_this_module():
    pass


def get_output_dir_for_test():
    return 'out-comptests'


try:
    from comptests import comptest as unit_test  # @UnusedImport
    from comptests import run_module_tests as run_tests_for_this_module  # @UnusedImport
    from comptests.comptests import get_comptests_output_dir as get_output_dir_for_test  # @UnusedImport

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

    def get_output_dir_for_test():
        return 'out-comptests'


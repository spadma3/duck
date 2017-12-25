from . import logger

__all__ = ['unit_test']


try: 
    from comptests import comptest as unit_test  # @UnusedImport
    logger.warning('Using the Comptests testing framework.')
    using_fake_tests = False
except ImportError:
    logger.warning('Unit tests are disabled becaused Comptests not found.')
    using_fake_tests = True
    
if using_fake_tests:
    def unit_test(f):
        return f

__all__ = ['logger']

import logging
logging.basicConfig()
logger = logging.getLogger('DT')
logger.setLevel(logging.DEBUG)
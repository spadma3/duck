

import logging
logging.basicConfig()
logger = logging.getLogger('DT')
logger.setLevel(logging.DEBUG)


# Let's make sure we have the depedencies

try:
    import comptests
except:
    msg = 'Comptests not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     pip install --user comptests'
    raise Exception(msg)
    
    
try:
    import procgraph  # @UnresolvedImport
except:
    msg = 'procgraph not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     pip install --user procgraph'
    raise Exception(msg)
    

try:
    import frozendict
except:
    msg = 'frozendict not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     sudo apt install python-frozendict'
    raise Exception(msg)


try:
    from ruamel import yaml
except:
    msg = 'ruamel.yaml not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     sudo apt install python-ruamel.yaml'
    raise Exception(msg)

from .constants import *

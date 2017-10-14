# -*- coding: utf-8 -*-

from collections import namedtuple

class ChecksConstants(object):
    FAIL = 'check failed'
    ERROR = 'INVALID TEST'
    OK = 'check passed'
    SKIP = 'skipped'
    statuses = [FAIL, ERROR, OK, SKIP]
    
    VISUALIZE_SYMBOLS = {
        OK: u'✓'.encode('utf8'), #✔',
        FAIL: u'✗'.encode('utf8'),
#         OK: 'ok',
#         FAIL: 'fail',
        SKIP: 'skip',
        ERROR: '!', 
    }


Result = namedtuple('Result', 'entry status out_short out_long')

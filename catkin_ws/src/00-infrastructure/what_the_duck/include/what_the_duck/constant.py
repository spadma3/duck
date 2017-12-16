# -*- coding: utf-8 -*-

from collections import namedtuple


class ChecksConstants(object):
    FAIL = 'failed' # do not change
    ERROR = 'invalid' # do not change
    OK = 'passed'# do not change
    SKIP = 'skipped'# do not change
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

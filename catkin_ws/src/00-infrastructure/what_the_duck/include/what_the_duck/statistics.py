# -*- coding: utf-8 -*-
from duckietown_utils import indent_with_label
from  .constant import ChecksConstants
from .visualize import MakeSpans 
from contracts.utils import indent

class Statistics(object):
    def __init__(self, results):
        self.ok = [_ for _ in results if _.status in [ChecksConstants.OK] ]
        self.failed = [_ for _ in results if _.status in [ChecksConstants.FAIL] ]
        self.errored = [_ for _ in results if _.status in [ChecksConstants.ERROR] ]
        self.skipped = [_ for _ in results if _.status in [ChecksConstants.SKIP] ]
        self.nfailures = len(self.failed + self.errored)

def display_short_statistics(results):
    stats = Statistics(results)
    NL = '\n'
    o = 'Summary of results:' + NL
    M = MakeSpans()
    o += M.green('* %d test(s) executed successfully.' % len(stats.ok)) + NL
    
    if stats.skipped:
        o += M.yellow('* %d test(s) were skipped.' % len(stats.skipped)) + NL
    else:
        o += '* No tests were skipped.' + NL
    
    if stats.failed:
        o += M.red('* %d test(s) failed\n' % len(stats.failed))
    else:
        o += M.green('* All tests executed succeeded.') + NL
        
    if stats.errored:
        o += M.red('* Found %d invalid test(s) [what-the-duck bugs]' % len(stats.errored)) + NL
        msg = ("Note that these are *bugs* in `what-the-duck` or its plugins "
               "and should be reported \nas soon as possible at the tracker: \n\n"
               " https://github.com/duckietown/Software/issues")
        o += M.bright_red(msg) + NL * 2
        
    return o
            
def display_summary(results): 
    stats = Statistics(results)
    NL = '\n'
    o = 'Summary of results:' + NL
    M = MakeSpans()
    o += M.green('* %d test(s) executed successfully.' % len(stats.ok)) + NL
    if stats.skipped:
        o += M.yellow('* %d test(s) were skipped.' % len(stats.skipped)) + NL
    else:
        o += '* No tests were skipped.' + NL
    
    if stats.nfailures == 0:
        o += M.green('* All tests executed succeeded.') + NL
    else:
        if stats.failed:        
            o += M.red('* Found %d failure(s):\n' % len(stats.failed)) + NL
            failed_sorted = sort_by_type(stats.failed)
            for i, f in enumerate(failed_sorted):
                # Skip a line for each group
                if i and (get_type(failed_sorted[i]) != get_type(failed_sorted[i-1])):
                    o += NL
                s = '  - %12s: %s : %s' % (
                    M.red(f.status), 
                    f.entry.desc, 
                    M.red(f.out_short))
                o += s + NL
            o += NL
            
        if stats.errored:
            o += M.red('* Found %d invalid test(s):\n' % len(stats.errored))
            for f in sort_by_type(stats.errored):
                s = '  - %12s:   %s' % (f.status, f.entry.desc)
                o += M.bright_red(s) + NL
            o += NL
            msg = ("Note that these are *bugs* in `what-the-duck` or its plugins "
                   "and should be reported \nas soon as possible at the tracker: \n\n"
                   " https://github.com/duckietown/Software/issues")
            o += M.bright_red(msg) + NL * 2

                
        o += M.red('\nSee above for details and suggestions.')
    res = '<pre><code>%s</code></pre>' %  o
    return res

def get_type(x):
    return type(x.entry.check).__name__ 

def sort_by_type(results):
    return sorted(results, key=get_type)


def display_results(results, show_successes):
    """ Returns a string """
    o = "" 
    
    for r in results:
        if not show_successes and r.status in [ChecksConstants.OK]:
            continue
        o += display_results_one(r)
        
    res = '<pre><code>%s</code></pre>' %  o
    return res
        
def display_results_one(r):
    M = MakeSpans()
    
    def L(s):
        return s.rjust(20) + '  '
    
    NL = '\n'
    o = ''
    symbol = ChecksConstants.VISUALIZE_SYMBOLS[r.status]
    s = '%05s  %s ' % (symbol, r.entry.desc)
    if r.status in [ChecksConstants.OK]:
        o += M.green(s) + NL
        if r.out_long:
            o += NL + indent(r.out_long, '     > ') + NL + NL
            
    elif r.status in [ChecksConstants.FAIL]:
        o += NL
        o += M.red(s) + NL
        o += M.red(indent_with_label(r.out_short, L(' failure:'))) + NL
        if r.out_long:
            o += M.red(indent_with_label(r.out_long, L('details:'))) + NL
        if r.entry.diagnosis is not None:
            s = str(r.entry.diagnosis)
            s = indent_with_label(s, L('diagnosis:'))
            o += s + NL*2
        for resolution in r.entry.resolutions:
            s = str(resolution)
            s = indent_with_label(s, L('resolution:'))
            o += s + NL*2
        
    elif r.status in [ChecksConstants.ERROR]:
        o += M.bright_red(s) + NL
        o += M.bright_red(indent_with_label(r.out_short, L(' error:'))) + NL
        if r.out_long:
            o += M.bright_red(indent_with_label(r.out_long, L('details:'))) + NL
        o += NL
        
    elif r.status in [ChecksConstants.SKIP]:
        o += M.yellow(s) + NL
        o += M.yellow(indent_with_label(r.out_short, L(' reason:'))) + NL
        if r.out_long:
            o += M.yellow(indent_with_label(r.out_long, L('details:'))) + NL
        o += NL
    else:
        assert False, r.status
    return o

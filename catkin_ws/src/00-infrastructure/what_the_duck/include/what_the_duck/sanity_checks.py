# -*- coding: utf-8 -*-
from collections import namedtuple
import sys
import traceback

from termcolor import cprint, colored

from duckietown_utils import logger
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.system_cmd_imp import indent_with_label
from what_the_duck.check import CheckError, CheckFailed

from .list_of_checks import get_checks


FAIL = 'check failed'
ERROR = 'INVALID TEST'
OK = 'check passed'
SKIP = 'skipped'
statuses = [FAIL, ERROR, OK, SKIP]

VISUALIZE_SYMBOLS = {
    OK: '✓', #✔',
    FAIL: '✗',
    SKIP: 'skip',
    ERROR: '!', 
}


Result = namedtuple('Result', 'entry status out_short out_long')

WTD = colored('what_the_duck', 'cyan', attrs=['bold'])

def do_all_checks():
    print("\n%s checks many things about the Duckiebot configuration.\n" % WTD)
    entries = get_checks()
    print('%s will run %s tests.\n' % (WTD, len(entries))) 
    results = run_checks(entries)
    display_results(results)
    ok = [_ for _ in results if _.status in [OK] ]
    failed = [_ for _ in results if _.status in [FAIL] ]
    errored = [_ for _ in results if _.status in [ERROR] ]
    skipped = [_ for _ in results if _.status in [SKIP] ]
    nfailures = len(failed + errored)
    
    print('Summary of results:')
    
    print_green('* %d test(s) executed successfully.' % len(ok))
    if skipped:
        print_yellow('* %d test(s) were skipped.' % len(skipped))
    else:
        print('* No tests were skipped.')
    
    if nfailures == 0:
        print_green('* All tests executed succeeded.')
        bye()
        sys.exit(0)
    else:

        if failed:        
            print_red('* Found %d failure(s):\n' % len(failed))
            failed_sorted = sort_by_type(failed)
            for i, f in enumerate(failed_sorted):
                # Skip a line for each group
                if i and (get_type(failed_sorted[i]) != get_type(failed_sorted[i-1])):
                    print('')
                s = '  - %12s: %s: %s' % (
                    colored(f.status, 'red'), 
                    f.entry.desc, 
                    colored(f.out_short, 'red'))
                print(s)
            print_red('')
            
        if errored:
            print_bright_red('* Found %d invalid test(s):\n' % len(errored))
            for f in sort_by_type(errored):
                s = '  - %12s:   %s' % (f.status, f.entry.desc)
                print_bright_red(s)
            print_bright_red('')
            msg = ("Note that these are *bugs* in `what-the-duck` or its plugins "
                   "and should be reported \nas soon as possible at the tracker: \n\n"
                   " https://github.com/duckietown/Software/issues")
            print_bright_red(msg)
            print_bright_red('')
                
        print_red('\nSee above for details and suggestions.')
        bye()
        sys.exit(nfailures)

def get_type(x):
    return type(x.entry.check).__name__ 
    
def sort_by_type(results):
    return sorted(results, key=get_type)

def bye():
    print('\nPlease add all tests that can be automated to %s.' % WTD)
        
def run_checks(entries):
    """ Returns the names of the failures  """
    results = [] 
    
    def record_result(r):
        results.append(r) 
    
    # raise NotRun if not previously run
    class NotRun(Exception): pass
    
    def get_previous_result_status(e):
        for r in results:
            if e == r.entry:
                return r.status
            
        logger.error('Could not find %s' % e)
        logger.error(results)
        raise NotRun()
    
    for entry in entries:
        
        # check dependencies
        only_run_if = entry.only_run_if
        if only_run_if is None:
            pass
        else:
            try:
                dep_status = get_previous_result_status(only_run_if)
            
                if dep_status in [FAIL, ERROR]:
                    msg = "Skipped because the previous test %r failed." % (only_run_if.desc)
                    r = Result(entry=entry, status=SKIP, out_short=msg, out_long='')
                    record_result(r)
                    continue
                
                elif dep_status in [SKIP]:
                    msg = "Skipped because the previous test %r skipped." % (only_run_if.desc)
                    r = Result(entry=entry, status=SKIP, out_short=msg, out_long='')
                    record_result(r)
                    continue

            except NotRun:
                msg = 'Dependency did not run yet.'
                r = Result(entry=entry, status=ERROR, out_short=msg, out_long='', )
                record_result(r)
                continue
        
        # at this point, either it's None or passed
        assert only_run_if is None or (get_previous_result_status(only_run_if) == OK)
    
        try:
            res = entry.check.check() or ''
            r = Result(entry=entry, status=OK, out_short=res, out_long='')
            record_result(r)
            
        except CheckError as e:
            r = Result(entry=entry, status=ERROR, 
                       out_short='Could not run test.',
                       out_long=e.long_explanation)
            record_result(r)
            
        except CheckFailed as e:
            r = Result(entry=entry, status=FAIL, 
                       out_short=e.compact,
                       out_long=e.long_explanation)
            record_result(r)
            
        except Exception as e:
            msg = 'Invalid test: it raised the exception %s.' % type(e).__name__
            l = 'I expect the tests to only raise CheckError or CheckFailed.'
            l += '\n\nEntire exception:\n\n'
            l += indent(traceback.format_exc(e), '  ')
            r = Result(entry=entry, status=ERROR, 
                       out_short=msg,
                       out_long=l)
            record_result(r)
            
    return results

def print_green(s):
    cprint(s, 'green')

def print_red(s):
    cprint(s, 'red')

def print_bright_red(s):
    cprint(s, 'red', attrs=['bold'])
        
def print_yellow(s):
    cprint(s, 'yellow')

def display_results(results): 
    def L(s):
        return s.rjust(20) + '  '

    for r in results:
        symbol = VISUALIZE_SYMBOLS[r.status]
        s = '%05s  %s ' % (symbol, r.entry.desc)
        if r.status in [OK]:
            print_green(s)
        elif r.status in [FAIL]:
            print('')
            print_red(s)
            print_red(indent_with_label(r.out_short, L(' failure:')))
            if r.out_long:
                print_red(indent_with_label(r.out_long, L('details:')))
            if r.entry.diagnosis is not None:
                s = str(r.entry.diagnosis)
                s = indent_with_label(s, L('diagnosis:'))
                print(s)
                print('')
            for resolution in r.entry.resolutions:
                s = str(resolution)
                s = indent_with_label(s, L('resolution:'))
                print(s)
                print('')
            
        elif r.status in [ERROR]:
            print_bright_red(s)
            print_bright_red(indent_with_label(r.out_short, L(' error:')))
            if r.out_long:
                print_bright_red(indent_with_label(r.out_long, L('details:')))
            print('')
            
        elif r.status in [SKIP]:
            print_yellow(s)
            print_yellow(indent_with_label(r.out_short, L(' reason:')))
            if r.out_long:
                print_yellow(indent_with_label(r.out_long, L('details:')))
            print('')
        else:
            assert False, r.status
            
    

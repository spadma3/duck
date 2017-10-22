# -*- coding: utf-8 -*-
import getpass
import socket
import sys
import traceback

from termcolor import colored

from duckietown_utils import indent, logger, write_data_to_file

from .check import CheckError, CheckFailed
from .constant import ChecksConstants, Result
from .list_of_checks import get_checks
from .statistics import display_results, display_summary, Statistics
from .visualize import escaped_from_html
from .statistics import display_short_statistics
from .mongo_suppor import upload_results


def do_all_checks():
    username = getpass.getuser()
    hostname = socket.gethostname()
    filename = 'what_the_duck-%s-%s.html' % (hostname, username)
    WTD = colored('what_the_duck', 'cyan', attrs=['bold'])
    entries = get_checks()
    
#     logger.info("%s checks many things about the Duckiebot configuration" % WTD)
    logger.info('%s will run %s tests.' % (WTD, len(entries))) 
    results = run_checks(entries)
     
    o_complete = ""
    o_complete += display_results(results, show_successes=True) + '\n\n'
    o_complete += display_summary(results) + '\n\n'
    
    o_user = ""
    o_user += display_results(results, show_successes=False)
    o_user += '\n\n' + display_short_statistics(results)
    
    print(escaped_from_html(o_user))
    
    write_data_to_file(o_user, filename)
#     print('\nNow send the file "%s" to the TA/instructors.' % filename)
    
    upload_results(results)
#     
#     print('\nYou can also upload it using the following command: ')
#     print('\n  scp %s duckiestats@frankfurt.co-design.science:%s ' % (filename, filename))
    stats = Statistics(results)
    if stats.nfailures == 0:    
        sys.exit(0)
    else:
        sys.exit(stats.nfailures)


        
def run_checks(entries):
    """ Returns the names of the failures  """
    results = [] 
    
#     entries = entries[:10]
    
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
            
                if dep_status in [ChecksConstants.FAIL, ChecksConstants.ERROR]:
                    msg = "Skipped because the previous test %r failed." % (only_run_if.desc)
                    r = Result(entry=entry, status=ChecksConstants.SKIP, out_short=msg, out_long='')
                    record_result(r)
                    continue
                
                elif dep_status in [ChecksConstants.SKIP]:
                    msg = "Skipped because the previous test %r skipped." % (only_run_if.desc)
                    r = Result(entry=entry, status=ChecksConstants.SKIP, out_short=msg, out_long='')
                    record_result(r)
                    continue

            except NotRun:
                msg = 'Dependency did not run yet.'
                r = Result(entry=entry, status=ChecksConstants.ERROR, out_short=msg, out_long='', )
                record_result(r)
                continue
        
        # at this point, either it's None or passed
        assert only_run_if is None or (get_previous_result_status(only_run_if) == ChecksConstants.OK)
    
        try:
            res = entry.check.check() or ''
            r = Result(entry=entry, status=ChecksConstants.OK, out_short='', out_long=res)
            record_result(r)
            
        except CheckError as e:
            r = Result(entry=entry, status=ChecksConstants.ERROR, 
                       out_short='Could not run test.',
                       out_long=e.long_explanation)
            record_result(r)
            
        except CheckFailed as e:
            r = Result(entry=entry, status=ChecksConstants.FAIL, 
                       out_short=e.compact,
                       out_long=e.long_explanation)
            record_result(r)
            
        except Exception as e:
            msg = 'Invalid test: it raised the exception %s.' % type(e).__name__
            l = 'I expect the tests to only raise CheckError or CheckFailed.'
            l += '\n\nEntire exception:\n\n'
            l += indent(traceback.format_exc(e), '  ')
            r = Result(entry=entry, status=ChecksConstants.ERROR, 
                       out_short=msg,
                       out_long=l)
            record_result(r)
            
    return results

    
    

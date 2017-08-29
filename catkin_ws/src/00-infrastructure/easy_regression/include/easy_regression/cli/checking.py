from datetime import datetime
import getpass
import socket

from duckietown_utils.constants import get_duckietown_root
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.system_cmd_imp import contract, system_cmd_result
from easy_regression.conditions.interface import CheckResult, RTCheck
from easy_regression.conditions.result_db import ResultDBEntry, ResultDB
from easy_regression.cli.db_yaml import get_unique_filename, yaml_from_rdbe
import os
from duckietown_utils.file_utils import write_data_to_file
from duckietown_utils.dates import format_datetime_as_YYYY_MM_DD
from easy_algo.algo_db import get_easy_algo_db


def git_cmd(cmd):
    cwd = get_duckietown_root()
    res = system_cmd_result(cwd, cmd,
              display_stdout=False,
              display_stderr=False,
              raise_on_error=True)
    return res.stdout.strip()
    

def make_entry(rt_name, results_all):
    user = getpass.getuser()
    hostname = socket.gethostname()
    date = format_datetime_as_YYYY_MM_DD(datetime.now())
    import platform
    cpu = platform.processor()
    branch = git_cmd('git rev-parse --abbrev-ref HEAD')
    commit = git_cmd('git rev-parse --verify HEAD')
    current = ResultDBEntry(regression_test_name=rt_name, 
                            date=date,
                            host=hostname,
                            cpu=cpu,
                            user=user,
                            results=results_all,
                            branch=branch,
                            commit=commit)
    return current 

def compute_check_results(rt_name, rt, results_all):
    current = make_entry(rt_name, results_all)
    
    algo_db = get_easy_algo_db()
    entries_names = algo_db.query('rdbe', 'parameters:regression_test_name:%s'%rt_name)
    print('entries: %s' % list(entries_names))
    entries = []
    for name in entries_names:
        e = algo_db.create_instance('rdbe', name)
        entries.append(e)
    
    rdb = ResultDB(current=current, entries=entries)
    
    res = []
    for cwc in rt.get_checks():
        for check in cwc.checks:
            r = check.check(rdb)
            assert isinstance(r, CheckResult)
            res.append(r)
    return res
        
def display_check_results(results, out):
    s = ""
    for i, r in enumerate(results):
        s += '\n' + indent(str(r), '', '%d of %d: ' % (i+1, len(results)))
    print(s)
    
def write_to_db(rt_name, results_all, out):
    rdbe = make_entry(rt_name, results_all)
    fn = get_unique_filename(rt_name, rdbe)
    s = yaml_from_rdbe(rdbe)
    print s
    filename = os.path.join(out, fn)
    write_data_to_file(s, filename)
    
    
@contract(results='list($CheckResult)')
def fail_if_not_expected(results, expect):
    statuses = [r.status for r in results]
    summary = summarize_statuses(statuses)
    if summary != expect:
        msg = 'Expected status %r, but got %r.' % (expect, summary)
        for i, r in enumerate(results):
            msg += '\n' + indent(str(r), '', '%d of %d: ' % (i+1, len(results)))
        raise Exception(msg)
    
def summarize_statuses(codes):
    # abnormal + ... = abnormal
    # fail + .. = fail
    # notfound + ... = notfound
    for c in codes:
        assert c in RTCheck.CHECK_RESULTS
    if RTCheck.ABNORMAL in codes:
        return RTCheck.ABNORMAL
    if RTCheck.FAIL in codes:
        return RTCheck.FAIL
    if RTCheck.NODATA in codes:
        return RTCheck.NODATA
    for c in codes:
        assert c == RTCheck.OK
    return RTCheck.OK
        
        
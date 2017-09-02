from comptests.registrar import comptest, run_module_tests

from duckietown_utils import fuzzy_match
from easy_logs.logs_db import get_easy_logs_db_cached_if_possible
from easy_logs.time_slice import filters_slice


@comptest
def parse_expressions():
    db = get_easy_logs_db_cached_if_possible()
    logs = db.logs
    one = logs.keys()[0]
#     l0 = logs[one]
#     print yaml_dump_pretty(l0._asdict())
    query = one + '/{10:15}'
    res = fuzzy_match(query, logs, filters=filters_slice, raise_if_no_matches=True)
    
    assert len(res) == 1
    l1 = res[one]
    assert l1.t0 == 10
    assert l1.t1 == 15
    assert l1.length == 5, l1.length
    
 
    query = one + '/{10:15}/{1:3}'
    res2 = fuzzy_match(query, logs, filters=filters_slice, raise_if_no_matches=True)
    assert len(res2) == 1
    l2 = res2[one]
#     print l2.t0, l2.t1, l2.length
    assert l2.t0 == 11
    assert l2.t1 == 13
    assert l2.length == 2, l1.length
    
    
@comptest
def parse_expressions2():
    db = get_easy_logs_db_cached_if_possible()
    logs = db.logs
    one = logs.keys()[0]
    query = one + '/{10.5:15.5}'
    res = fuzzy_match(query, logs, filters=filters_slice, raise_if_no_matches=True)
    
    assert len(res) == 1
    l1 = res[one]
    assert l1.t0 == 10.5
    assert l1.t1 == 15.5
    assert l1.length == 5, l1.length
    
if __name__ == '__main__':
    run_module_tests()
    
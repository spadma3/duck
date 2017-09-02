
from collections import OrderedDict
import os

from quickapp import QuickApp

from duckietown_utils import DTUserError
from duckietown_utils import contract
from duckietown_utils import create_tmpdir
from duckietown_utils import d8n_make_video_from_bag
from duckietown_utils import logger
from duckietown_utils.cli import D8AppWithLogs
from easy_algo.algo_db import get_easy_algo_db
from easy_logs.cli.require import get_log_if_not_exists
from easy_regression.cli.analysis_and_stat import job_analyze, job_merge, print_results
from easy_regression.cli.checking import compute_check_results, display_check_results, fail_if_not_expected,\
    write_to_db
from easy_regression.cli.processing import process_one
from easy_regression.conditions.interface import RTCheck
from easy_regression.regression_test import RegressionTest


ALL_LOGS = 'all'

class RunRegressionTest(D8AppWithLogs, QuickApp): 
    """ Run regression tests. """

    def define_options(self, params):
        g = 'Running regressions tests'
        params.add_string('tests', help="Query for tests instances.", group=g)
        
        h = 'Expected status code for this regression test; one of: %s' % ", ".join(RTCheck.CHECK_RESULTS)
        default = RTCheck.OK
        params.add_string('expect', help=h, group=g, default=default)  
        
    def define_jobs_context(self, context):
        easy_algo_db = get_easy_algo_db()
        
        expect = self.options.expect
        
        if not expect in RTCheck.CHECK_RESULTS:
            msg = 'Invalid expect status %s; must be one of %s.' % (expect, RTCheck.CHECK_RESULTS)
            raise DTUserError(msg)
        
        query = self.options.tests
        regression_tests = easy_algo_db.query('regression_test', query, raise_if_no_matches=True)
        
        for rt_name in regression_tests:
            rt = easy_algo_db.create_instance('regression_test', rt_name)
            
            easy_logs_db = self.get_easy_logs_db()
            c = context.child(rt_name)
            
            outd = os.path.join(self.options.output, 'regression_tests', rt_name)
            jobs_rt(c, rt_name, rt, easy_logs_db, outd, expect) 

@contract(rt=RegressionTest)
def jobs_rt(context, rt_name, rt, easy_logs_db, out, expect):
    
    logs = rt.get_logs(easy_logs_db)
    
    processors = rt.get_processors()
    
    analyzers = rt.get_analyzers()
    
    logger.info('logs: %s' % list(logs))
    logger.info('processors: %s' % processors)
    logger.info('analyzers: %s' % analyzers)
    
    # results_all['analyzer'][log_name]
    results_all = {}
    for a in analyzers:
        results_all[a] = OrderedDict()
    
    tmpdir = create_tmpdir()
    for log_name, log in logs.items():
        c = context.child(log_name)
        # process one     
        log_out = os.path.join(tmpdir, 'logs', log_name + '/'  + 'out.bag')
        bag_filename = c.comp(get_log_if_not_exists, easy_logs_db.logs, log_name)
        t0 = log.t0
        t1 = log.t1
        log_out_ = c.comp(process_one, bag_filename, t0, t1, processors, log_out, job_id=log_name)
        
        for a in analyzers:
            results_all[a][log_name] = c.comp(job_analyze, log_out_, a, job_id=a) 
        
        for topic in rt.get_topic_videos():
            mp4 = os.path.join(out, 'videos', log_name, topic + '.mp4')
            c.comp(d8n_make_video_from_bag, log_out_, topic, mp4)

    for a in analyzers:
        results_all[a][ALL_LOGS] = context.comp(job_merge, results_all[a], a)
    
    context.comp(print_results, analyzers, results_all, out)

    check_results = context.comp(compute_check_results, rt_name, rt, results_all)
    context.comp(display_check_results, check_results, out)
    
    context.comp(fail_if_not_expected, check_results, expect)
    context.comp(write_to_db, rt_name, results_all, out)
    

    

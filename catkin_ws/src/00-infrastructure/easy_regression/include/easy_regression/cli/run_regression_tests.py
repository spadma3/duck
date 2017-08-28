from collections import OrderedDict
import os

from quickapp import QuickApp

from duckietown_utils import logger
from duckietown_utils.cli import D8AppWithLogs
from duckietown_utils.exceptions import wrap_script_entry_point
from duckietown_utils.system_cmd_imp import contract
from easy_algo.algo_db import get_easy_algo_db
from easy_regression.cli.analysis_and_stat import job_analyze, job_merge,\
    print_results
from easy_regression.cli.processing import process_one
from easy_regression.regression_test import RegressionTest
from easy_logs.cli.require import get_log_if_not_exists


ALL_LOGS = 'all logs'

class RunRegressionTest(D8AppWithLogs, QuickApp): 
    """ Run regression tests. """

    def define_options(self, params):
        g = 'Running regressions tests'
        params.add_string('tests', help="Query for tests instances.", group=g)  
        
    def define_jobs_context(self, context):
        easy_algo_db = get_easy_algo_db()
        
        query = self.options.tests
        regression_tests = easy_algo_db.query('regression_test', query)
        print(regression_tests)
        for r in regression_tests:
            rt = easy_algo_db.create_instance('regression_test', r)
            
            easy_logs_db = self.get_easy_logs_db()
            c = context.child(r)
            
            outd = os.path.join(self.options.output, 'regression_tests', r)
            jobs_rt(c, rt, easy_logs_db, outd) 

@contract(rt=RegressionTest)
def jobs_rt(context, rt, easy_logs_db, out):
    
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
    
    for log_name, log in logs.items():
        c = context.child(log_name)
        # process one 
        log_out = os.path.join(out, 'logs', log_name + '/'  + 'out.bag')
        bag_filename = c.comp(get_log_if_not_exists, easy_logs_db.logs, log_name)
        log_out_ = c.comp(process_one, bag_filename, processors, log_out, job_id=log_name)
        
        for a in analyzers:
            results_all[a][log_name] = c.comp(job_analyze, log_out_, a, job_id=a) 

    for a in analyzers:
        results_all[a][ALL_LOGS] = context.comp(job_merge, results_all[a], a)
    
    context.comp(print_results, analyzers, results_all, out)


    

    
def run_regression_test_main():
    wrap_script_entry_point(RunRegressionTest.get_sys_main())

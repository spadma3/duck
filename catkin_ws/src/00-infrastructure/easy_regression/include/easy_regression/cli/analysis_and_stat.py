from collections import OrderedDict
import os

from duckietown_utils import logger
from duckietown_utils.file_utils import write_data_to_file
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.system_cmd_imp import contract
from duckietown_utils.text_utils import format_table_plus
from duckietown_utils.yaml_pretty import yaml_dump_pretty
from easy_algo.algo_db import get_easy_algo_db
from easy_regression.analyzer_interface import AnalyzerInterface
import rosbag  # @UnresolvedImport


@contract(analyzers='list(str)')
def print_results(analyzers, results_all, out):
    base = os.path.join(out, 'statistics')
    yaml_data  = yaml_dump_pretty(results_all)
    write_data_to_file(yaml_data, os.path.join(base, 'statistics.yaml'))
    print(yaml_data)
    
    for a in analyzers:
        write_data_to_file(yaml_dump_pretty(results_all[a]), 
                           os.path.join(base, '%s.table.yaml' % a))
        s = ""
        s += '\n' + '-' * 10 + ' Results for %s ' % a + '-' * 10
        table = table_for_analyzer(results_all[a])
        s += '\n' + indent(format_table_plus(table, colspacing=3), '  ')
        s += '\n'
        write_data_to_file(yaml_data, os.path.join(base, '%s.table.txt' % a))
    
def table_for_analyzer(results_all):
    from easy_regression.cli.run_regression_tests import ALL_LOGS
    keys = list(results_all[ALL_LOGS].keys())
    head = ['log name'] + keys
    table = [head]
    for k, v in results_all.items():
        row = [k] + list(v.values())
        
        if k == ALL_LOGS:
            table.append(['']*len(head))
        table.append(row)
    
    return table

def job_merge(results, analyzer):
    """
        results: log name -> results dict
    """
    easy_algo_db = get_easy_algo_db()
    analyzer_instance = easy_algo_db.create_instance('analyzer', analyzer)
    
    results = list(results.values())

    total = merge_n(analyzer_instance, results)
    return total
     
@contract(analyzer=AnalyzerInterface)
def merge_n(analyzer, results):
    if len(results) == 1:
        return results[0]
    else:
        first=results[0]
        rest = merge_n(analyzer, results[1:])
        r = OrderedDict()
        analyzer.reduce(first, rest, r)
        return r 
    
@contract(analyzer=str)
def job_analyze(log, analyzer):
    easy_algo_db = get_easy_algo_db()
    analyzer_instance = easy_algo_db.create_instance('analyzer', analyzer)
    in_bag = rosbag.Bag(log)
    results = OrderedDict()
    logger.info('Running %s on %s' % (analyzer, log))
    analyzer_instance.analyze_log(in_bag, results)
    in_bag.close()
    return results
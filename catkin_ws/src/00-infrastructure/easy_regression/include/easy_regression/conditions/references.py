from contracts.utils import check_isinstance
import yaml

from duckietown_utils.exception_utils import raise_wrapped, check_is_in
from duckietown_utils.system_cmd_imp import contract
from duckietown_utils.text_utils import remove_prefix, string_split
from easy_regression.conditions.eval import Evaluable, EvaluationError
from easy_regression.conditions.interface import RTParseError
from easy_regression.conditions.result_db import ResultDBEntry


def parse_reference(s):
    """
        v:analyzer/log/statistic~master@date
        
    """
    prefix = 'v:'
    
    if s.startswith(prefix):
        s = remove_prefix(s, prefix)
        
        T_DATE = '@'
        T_BRANCH = '~'
        T_COMMIT = '?'
        TS = [T_DATE, T_BRANCH, T_COMMIT]
        
        if (T_COMMIT in s)  and (T_DATE in s):
            msg = 'Cannot specify commit and date: %s' % s
            raise RTParseError(msg)
        
        date = None
        commit = None
        branch_spec = None
        
        def get_last_one(s0):
            for c in s0[::-1]:
                if c in TS:
                    return c
                
        while True:
            which = get_last_one(s)
            
            if which is None:
                break
            elif which == T_DATE:
                s, date_spec = string_split(s, T_DATE)
                if not date_spec:
                    msg = 'Invalid date spec %r.' % date_spec 
                    raise RTParseError(msg)
                date = parse_date_spec(date_spec)
            elif which == T_BRANCH:
                s, branch_spec = string_split(s, T_BRANCH)
                if not branch_spec:
                    msg = 'Invalid branch spec %r.' % branch_spec 
                    raise RTParseError(msg)
            elif which == T_COMMIT:
                s, commit = string_split(s, T_COMMIT)
                if not commit:
                    msg = 'Invalid commit %r.' % branch_spec 
                    raise RTParseError(msg)
        
            
              
        tokens = s.split('/')
        if not len(tokens) >= 3:
            msg = 'Expected "analyzer/log/statistic"'
            raise RTParseError(msg)
        
        analyzer = tokens[0]
        log = tokens[1]
        statistic = tuple(tokens[2:]) 
        
            
        return StatisticReference(analyzer=analyzer, log=log, statistic=statistic, 
                                  branch=branch_spec, date=date, commit=commit)
    
    try:
        c = yaml.load(s)
        if isinstance(c, str) and '/' in c:
            msg = 'The syntax is "v:analyzer/log/statistic"'
            msg += '\nInvalid string: %r' % c
            raise RTParseError(msg)
        return Constant(c)
    except yaml.YAMLError:
        msg = 'Could not parse reference %s.' % s.__repr__()
        raise RTParseError(msg)

def parse_date_spec(d):
    from dateutil.parser import parse
    try:
        return parse(d)
    except ValueError as e:
        msg = 'Cannot parse date %s.' % d.__repr__()
        raise_wrapped(RTParseError, e, msg, compact=True)

class StatisticReference(Evaluable):
    
    @contract(statistic='seq(str)')
    def __init__(self, analyzer, log, statistic, branch, date, commit):
        self.analyzer = analyzer
        self.log = log
        self.statistic = statistic
        self.branch = branch
        self.date = date
        self.commit = commit
    
    def __str__(self):
        return ('StatisticReference(%s,%s,%s,%s,%s)' % 
                (self.analyzer, self.log, self.statistic, self.branch, self.date))
        
    def eval(self, rdb):
        db_entry = rdb.query_results_one(branch=self.branch,
                                  date=self.date,
                                  commit=self.commit)
        check_isinstance(db_entry, ResultDBEntry)
#         print('Results= %s' % db_entry.__repr__())
        results = db_entry.results
        check_is_in('analyzer', self.analyzer, results, EvaluationError)
        logs = results[self.analyzer]
        check_is_in('log', self.log, logs, EvaluationError)
        forlog = logs[self.log]
        val = eval_name(forlog, self.statistic)
        return val
    
@contract(name_tuple=tuple)
def eval_name(x, name_tuple):
    if not name_tuple:
        return x
    else:
        first = name_tuple[0]
        rest = name_tuple[1:]
        check_is_in('value', first, x, EvaluationError)
        xx = x[first]
        return eval_name(xx, rest)
    
class Constant(Evaluable):
    def __init__(self, x):
        self.x = x
    def eval(self, _test_results):
        return self.x
    def __repr__(self):
        return 'Constant(%s)' % self.x.__repr__()
    

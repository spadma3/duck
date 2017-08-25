from contracts.utils import check_isinstance
from duckietown_utils.wildcards import  wildcard_to_regexp
from duckietown_utils.system_cmd_imp import contract
from collections import OrderedDict
from duckietown_utils.instantiate_utils import indent
from abc import abstractmethod, ABCMeta
import yaml
from duckietown_utils import logger
from contracts.interface import describe_type


class Spec(object):
    __metaclass__ = ABCMeta
    def __init__(self, children):
        self.children = children
        
    def __str__(self):
        mine = type(self).__name__
        
        c = [indent(str(_), '', ' - ') for _ in self.children]
        return indent("\n".join(c), '', mine+'')
    
    @abstractmethod
    def match(self, k):
        pass
    
    def match_dict(self, stuff):
        res = OrderedDict()
        for k, v in stuff.items():
            if self.match(k):
                res[k] = v
        return res
    

class Or(Spec):
        
    def match(self, x):
        for option in self.children:
            if option.match(x):
                return True
        return False 
    
    def match_dict(self, seq):
        matches = OrderedDict()
#         children_answers = []
        for option in self.children:
            theirs = option.match_dict(seq)
            for k, v in theirs.items():
                if not k in matches:
                    matches[k] = v
#             children_answers.append(theirs)
#             
#         for k, v in seq.items():
#             ok = any(k in _ for _ in children_answers)
#             if ok:
#                 matches[k] = v 
        return matches
    
class And(Spec): 
        
    def match(self, x):
        for option in self.children:
            if not option.match(x):
                return False
        return True

    def match_dict(self, seq):
        matches = OrderedDict()
        children_answers = []
        for option in self.children:
            theirs = option.match_dict(seq)
            children_answers.append(theirs)
            
        for k, v in seq.items():
            ok = all(k in _ for _ in children_answers)
            if ok:
                matches[k] = v 
        return matches
    
class ByTag(Spec):
    def __init__(self, tagname, spec):
        if '*' in tagname:
            msg = 'Invalid tag %s.' % tagname.__repr__()
            raise ValueError(msg)
        Spec.__init__(self, [])
        self.tagname = tagname
        self.spec = spec
        
    def __str__(self):
        return indent(self.spec.__str__(), '', 
                      'attribute %s satisfies \n  ' % self.tagname)
    
    def match(self, x):
        if not hasattr(x, self.tagname):
            msg = ('The object of type %s does not have attribute "%s".' %
                 (type(x).__name__, self.tagname))
            msg += '\nThe available attributes are:\n  %s' % sorted(x.__dict__.keys())
            raise ValueError(msg)
        val = getattr(x, self.tagname)
        res = self.spec.match(val)
        return res
   
    def match_dict(self, seq):
        matches = OrderedDict()
        for k, v in seq.items():
            if self.match(v):
                matches[k] = v 
        return matches
    
class Constant(Spec):
    def __init__(self, s):
        self.s = yaml.load(s)
    def __str__(self):
        return 'is equal to %r' % self.s
    def match(self, x):
        return self.s == x
         
class Wildcard(Spec):
    def __init__(self, pattern):
        self.pattern = pattern
        self.regexp = wildcard_to_regexp(pattern)
    def __str__(self):
        return 'matches %s' % self.pattern
    def match(self, x):
        return isinstance(x, str) and self.regexp.match(x)
def value_as_float(x):
    try:
        return float(x)
    except Exception as e:
        logger.error('Cannot convert to float %r: %s' % (x, e))
        raise
    
class LT(Spec):
    def __init__(self, value):
        self.value = value
    
    def __str__(self):
        return 'less than %s' % self.value
    
    def match(self, x):
        if x is None:
            return False
        v = value_as_float(x) 
        return v < self.value

class GT(Spec):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return 'greater than %s' % self.value
    def match(self, x):
        if x is None:
            return False
        v = value_as_float(x)
        return v > self.value
    
@contract(s=str, returns=Spec)   
def parse_match_spec(s):
    """
        
        a, b:>10 or +
    """
    check_isinstance(s, str)
    if '+' in s:
        tokens = s.split('+')
        return Or(map(parse_match_spec, tokens))
    if ',' in s:
        tokens = s.split(',')
        return And(map(parse_match_spec, tokens))
    if ':' in s:
        i = s.index(':')
        tagname = s[:i]
        tagvalue = s[i+1:]
        return ByTag(tagname, parse_match_spec(tagvalue))
    if s.startswith('<'):
        value = float(s[1:])
        return LT(value)
    if s.startswith('>'):
        value = float(s[1:])
        return GT(value)
    if s == 'all':
        return Wildcard('*')
    if '*' in s:
        return Wildcard(s)
    return Constant(s) 
    
@contract(stuff=dict)
def fuzzy_match(spec, stuff):
    """
        spec: a string
        logs: a dict of logs
    """
    if not isinstance(stuff, dict):
        msg = 'Expectd an OrderedDict, got %s.' % describe_type(stuff)
        raise ValueError(msg)
    check_isinstance(stuff, dict)
    check_isinstance(spec, str)
    spec = parse_match_spec(spec)
    return spec.match_dict(stuff)


#     
# def fuzzy_match1(spec, stuff):
#     
#     if ':' in spec:
#         i = spec.index(':')
#         tagname = spec[:i]
#         tagvalue = spec[i+1:]
#         return match_by_tag(tagname, tagvalue, stuff)
#     return match_by_name(spec, stuff)
# 
# def match_by_tag(tagname, tagspec, stuff):
#     check_isinstance(tagname, str)
#     check_isinstance(tagspec, str)
#     check_isinstance(stuff, dict)
#     result = OrderedDict()
#     for k, v in stuff.items():
#         if hasattr(v, tagname):
#             val = getattr(v, tagname)
#             if match(tagspec, val):
#                 result[k]=v
#     return result    
#     
# def match_by_name(spec, stuff):
#     result = OrderedDict()
#     for k, v in stuff.items():
#         if match(spec, k):
#             result[k] = v
#     return result




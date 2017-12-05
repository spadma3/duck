from collections import OrderedDict
from duckietown_utils import indent
from duckietown_utils.fuzzy import Spec


class MakeTimeSlice(Spec):
    def __init__(self, spec, t0, t1):
        Spec.__init__(self, [spec])
        self.t0 = t0
        self.t1 = t1
    
    def __str__(self):
        s  = 'MakeTimeSlice  { %s : %s }' % (self.t0, self.t1)
        s += '\n' + indent(str(self.children[0]), '  ')
        return s
    
    def match(self, x):
        raise NotImplementedError()
   
    def match_dict(self, seq):
        results = self.children[0].match_dict(seq)
        matches = OrderedDict()
        for k, v in results.items():
            matches[k] = self.transform(v) 
        return matches
    
    def transform(self, log):
        if not log.valid:
            return log
        u0 = log.t0
        u1 = log.t1
        assert (u0 is not None) and  (u1 is not None), log
        assert u0 <= u1
        if self.t0 is not None:
            new_start = u0 + self.t0
        else:
            new_start = u0
        if self.t1 is not None:
            new_end = u0 + self.t1
        else:
            new_end = u1
        length = new_end-new_start 
        return log._replace(t0=new_start, t1=new_end, length=length)
    
def slice_time(m, spec):
    if m.group('t0') is not None:
        t0 = float(m.group('t0'))
    else:
        t0 = None
        
    if m.group('t1') is not None:
        t1 = float(m.group('t1'))
    else:
        t1 = None
    return MakeTimeSlice(spec, t0, t1)

# float = "[-+]?[0-9]*\.?[0-9]+"
filters_slice = {
    r'{(?P<t0>[-+]?[0-9]*\.?[0-9]+)?:(?P<t1>[-+]?[0-9]*\.?[0-9]+)?}': slice_time,
}

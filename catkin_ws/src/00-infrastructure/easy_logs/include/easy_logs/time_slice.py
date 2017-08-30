from duckietown_utils.fuzzy import Spec
from duckietown_utils.instantiate_utils import indent
from collections import OrderedDict

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
        u0 = log.t0
        u1 = log.t1
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
    t0 = float(m.group('t0'))
    t1 = float(m.group('t1'))
    return MakeTimeSlice(spec, t0, t1)

filters_slice = {
    r'{(?P<t0>\d+)?:(?P<t1>\d+)?}': slice_time,
}

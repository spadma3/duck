from comptests.registrar import comptest, run_module_tests
from easy_regression.conditions.binary import parse_binary 
from easy_regression.conditions.interface import RTParseError


@comptest
def parse_binary_check_good():
    good = ['==', '>=', '<', '<=', '>', '==[10%]']
    
    for g in good:
        f = parse_binary(g)
        f(10,20)

@comptest
def parse_binary_check_bad():
    bad = ['=', '!', '==[10%', '==[10]', '=[10%]', '=[%]', '==[-10%]']
    for b in bad:
        try:
            res = parse_binary(b)
        except RTParseError:
            pass
        else:
            msg = 'Expected DTParseError.'
            msg += '\nString: %r' % b
            msg += '\nReturns: %s' % res
            raise Exception(msg)    

if __name__ == '__main__':
    run_module_tests()

from collections import OrderedDict, namedtuple
from comptests.registrar import comptest, run_module_tests
from duckietown_utils.fuzzy import fuzzy_match, parse_match_spec
from duckietown_utils.instantiate_utils import indent

def expect(data, query, result_keys):
        
    spec = parse_match_spec(query)
    print '-----'
    print 'Query: %s' % query
    print indent(spec, '', 'Spec: ')
    res = fuzzy_match(query, data)
    if list(res) != list(result_keys):
        msg = 'Error:'
        msg += '\n Data: %s' % data
        msg += '\n Query: %s' % query
        msg += '\n Result: %s' % list(res)
        msg += '\n Expect: %s' % list(result_keys)
        raise Exception(msg)
    
@comptest
def matches():
    Species = namedtuple('Species', 'name size')
    data = OrderedDict([
        ('one',  Species('horse', 'large')),
        ('two',  Species('dog', 'medium')),
        ('three',  Species('cat', 'medium')) 
    ])
    
    expect(data, '*', ['one', 'two', 'three'])
    expect(data, 'one', ['one'])
    expect(data, 'two+one', ['two', 'one'])
    expect(data, '*o', ['two'])
    expect(data, 'o*', ['one'])


@comptest
def matches_tags():
    Species = namedtuple('Species', 'name size weight')
    data = OrderedDict([
        ('jeb',  Species('A big horse', 'large', 200)),
        ('fuffy',  Species('A medium dog', 'medium', 50)),
        ('ronny',  Species('A medium cat', 'medium', 30)), 
    ])
    
    expect(data, '*', ['jeb', 'fuffy', 'ronny'])

    expect(data, 'name:A big horse', ['jeb'])
    expect(data, 'name:*medium*', ['fuffy', 'ronny'])
    expect(data, 'name:*medium*,weight:>40', ['fuffy'])
    
    expect(data, 'name:*medium*,weight:<40', ['ronny'])
    expect(data, 'fuffy+ronny', ['fuffy','ronny'])
    expect(data, 'name:*dog', ['fuffy'])
    expect(data, 'name:*cat', ['ronny'])
    expect(data, 'name:*dog+name:*cat', ['fuffy','ronny'])
    
@comptest
def specs1():
    print parse_match_spec('ciao')
    print parse_match_spec('ciao+no')
    
if __name__ == '__main__':
    run_module_tests()
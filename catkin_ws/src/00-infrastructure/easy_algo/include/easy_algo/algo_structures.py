from collections import namedtuple

EasyAlgoFamily = namedtuple('EasyAlgoFamily', 
                      ['filename', 
                       'family_name', 
                       'interface', 
                       'description', 
                       'instances_pattern', 'instances', 'tests', 'tests_pattern'])

            
EasyAlgoTest = namedtuple('EasyAlgoTest',
                              ['family_name',
                               'test_name',
                               'description',
                               'filename', # where it was specified
                               'constructor', # how to instance it
                               'parameters'])


EasyAlgoInstance = namedtuple('EasyAlgoInstance',
                              ['family_name',
                               'instance_name',
                               'description',
                               'filename', # where it was specified
                               'constructor', # how to instance it
                               'parameters'])

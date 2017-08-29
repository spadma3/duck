import os
from types import NoneType

from duckietown_utils.caching import get_cached
from duckietown_utils.constants import DuckietownConstants
from duckietown_utils.exception_utils import check_is_in
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.fuzzy import fuzzy_match
from duckietown_utils.instantiate_utils import import_name, instantiate, indent
from duckietown_utils.system_cmd_imp import contract
from duckietown_utils.text_utils import id_from_basename_pattern
from duckietown_utils.type_checks import dt_check_isinstance
from duckietown_utils.yaml_wrap import interpret_yaml_file, look_everywhere_for_config_files, get_config_sources,\
    look_everywhere_for_config_files2

from .algo_structures import EasyAlgoTest, EasyAlgoInstance, EasyAlgoFamily


__all__ = ['get_easy_algo_db', 'EasyAlgoDB']


def get_easy_algo_db():
    if EasyAlgoDB._singleton is None:
        cache_algos = DuckietownConstants.use_cache_for_algos
        EasyAlgoDB._singleton = get_cached('EasyAlgoDB', EasyAlgoDB) if cache_algos else EasyAlgoDB()
    return EasyAlgoDB._singleton

class EasyAlgoDB():
    _singleton = None 
    
    pattern = '*.easy_algo_family.yaml'
    
    @contract(sources='None|seq(str)')
    def __init__(self, sources=None):
        if sources is None:
            sources = get_config_sources()
        self.all_yaml = look_everywhere_for_config_files('*.yaml', sources)
        
        self.family_name2config = load_family_config(self.all_yaml)
    
    def query(self, family_name, query, raise_if_no_matches=False):
        family = self.get_family(family_name)
        instances = family.instances
        result = fuzzy_match(query, instances, raise_if_no_matches=raise_if_no_matches)
        return result
        
    def get_family(self, x):
        check_is_in('family', x, self.family_name2config)
        return self.family_name2config[x]
    
    def create_instance(self, family_name, instance_name):
        family = self.get_family(family_name)
        if not family.valid:
            msg = 'Cannot instantiate %r because its family %r is invalid.' % (instance_name, family_name)
            raise DTConfigException(msg)
            
        check_is_in('instance', instance_name, family.instances)
        instance = family.instances[instance_name]
        
        if not instance.valid:
            msg = ('Cannot instantiate because it is invalid:\n%s' % 
                   indent(instance.error_if_invalid, '> '))
            raise DTConfigException(msg)
        res = instantiate(instance.constructor, instance.parameters)
        
        interface = import_name(family.interface)
        if not isinstance(res, interface):
            msg = ('I expected that %r would be a %s but it is a %s.' % 
                   (instance_name, interface.__name__, type(res).__name__))
            raise DTConfigException(msg)
             
        return res
        
#         """ Instantiates an algorithm """
        
def load_family_config(all_yaml):
    """
        # now, for each family, we look for tests, which have name
        #  
        #     ![ID].![family_name]_test.yaml
        #
        # and configuration files, which are:
        #
        #     ![ID].![family_name]_test.yaml
        #
        #   
    """
    if not isinstance(all_yaml, dict):
        msg = 'Expected a dict, got %s' % type(all_yaml).__name__
        raise ValueError(msg)
    family_name2config = {}
    
    configs = look_everywhere_for_config_files2(EasyAlgoDB.pattern, all_yaml)
    for filename, contents in configs.items():
        c = interpret_yaml_file(filename, contents, interpret_easy_algo_config)

        if c.family_name in family_name2config:
            one = family_name2config[c.family_name].filename
            two = c.filename
            msg = 'Repeated filename:\n%s\n%s' % (one, two)
            raise DTConfigException(msg)

        def interpret_test_spec(filename, data):
            basename = os.path.basename(filename)
            test_name = id_from_basename_pattern(basename, c.tests_pattern)
            
            description = data.pop('description')
            dt_check_isinstance('description', description, str) 

            constructor = data.pop('constructor')
            dt_check_isinstance('constructor', constructor, str) 

            parameters = data.pop('parameters')
            dt_check_isinstance('parameters', parameters, (dict, NoneType))
            if parameters is None: parameters = {} 

            return EasyAlgoTest(family_name=c.family_name, test_name=test_name,
                                description=description, filename=filename,
                                constructor=constructor, parameters=parameters,
                                valid=True, error_if_invalid=None)
            
        def interpret_instance_spec(filename, data):
            basename = os.path.basename(filename)
            instance_name = id_from_basename_pattern(basename, c.instances_pattern)
            
            description = data.pop('description')
            dt_check_isinstance('description', description, str) 

            constructor = data.pop('constructor')
            dt_check_isinstance('constructor', constructor, str) 

            parameters = data.pop('parameters')
            dt_check_isinstance('parameters', parameters, (dict, NoneType))
            if parameters is None: parameters = {} 

            return EasyAlgoInstance(family_name=c.family_name, instance_name=instance_name,
                                    description=description, filename=filename,
                                    constructor=constructor, parameters=parameters,
                                    valid=True, error_if_invalid=None)
            
        tests = {}
        
        c = check_validity_family(c)
        
        _ = look_everywhere_for_config_files2(c.tests_pattern, all_yaml)
        for filename, contents in _.items():
            t = interpret_yaml_file(filename, contents, interpret_test_spec)
            if t.test_name in tests:
                one = tests[t.test_name].filename
                two = t.filename
                msg = 'Repeated filename:\n%s\n%s' % (one, two)
                raise DTConfigException(msg)
            
            t = check_validity_test(c, t)
            tests[t.test_name] = t
            
        instances = {}
        _ = look_everywhere_for_config_files2(c.instances_pattern, all_yaml)
        for filename, contents in _.items():
            i = interpret_yaml_file(filename, contents, interpret_instance_spec)
            if i.instance_name in instances:
                one = instances[i.instance_name].filename
                two = i.filename
                msg = 'Repeated filename:\n%s\n%s' % (one, two)
                raise DTConfigException(msg)
            
            i = check_validity_instance(c, i)
            instances[i.instance_name] = i
        
        c = c._replace(tests=tests, instances=instances)
        
        
        family_name2config[c.family_name] = c
        
    return family_name2config

@contract(f=EasyAlgoFamily, i=EasyAlgoInstance, returns=EasyAlgoInstance)
def check_validity_instance(f, i):
    if not f.valid:
        msg = 'Instance not valid because family not valid.'
        return i._replace(valid=False, error_if_invalid=msg)
     
    try:
        res = instantiate(i.constructor, i.parameters)
    except Exception as e:
        msg = str(e)
        return i._replace(valid=False, error_if_invalid=msg)
    
    interface = import_name(f.interface)
    
    if not isinstance(res, interface):
        msg = ('Expected a %s but it is a %s.' % 
               (interface.__name__, type(res).__name__))
        return i._replace(valid=False, error_if_invalid=msg)
    return i



@contract(f=EasyAlgoFamily, t=EasyAlgoInstance, returns=EasyAlgoInstance)
def check_validity_test(f, t):
    return t
        
@contract(f=EasyAlgoFamily, returns=EasyAlgoFamily)
def check_validity_family(f):
    f = check_validity_family_interface(f)
    return f

def check_validity_family_interface(f):
    # try to import interface
    symbol = f.interface
    try:
        import_name(symbol)
    except ValueError:
        #logger.error(e)
        error_if_invalid = 'Invalid symbol %r.' % symbol
        return f._replace(valid=False, error_if_invalid=error_if_invalid)
    return f
        
def interpret_easy_algo_config(filename, data):
    basename = os.path.basename(filename)
    family_name = id_from_basename_pattern(basename, EasyAlgoDB.pattern)
    instances_pattern = '*.%s.yaml' % family_name
    tests_pattern = '*.%s_test.yaml' % family_name
    
    dt_check_isinstance('contents', data, dict)
    
    
    description = data.pop('description')
    dt_check_isinstance('description', description, str) 
    
    interface = data.pop('interface')
    dt_check_isinstance('interface', interface, str) 
    
    tests = None
    instances = None 
    
    return EasyAlgoFamily(interface=interface, 
                            family_name=family_name,
                            filename=filename, 
                            tests=tests, 
                            tests_pattern = tests_pattern,
                            instances=instances,
                            instances_pattern=instances_pattern,
                            description=description,
                            valid=True,
                            error_if_invalid=False)
    




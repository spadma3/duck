import os

from duckietown_utils.caching import get_cached
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.text_utils import id_from_basename_pattern
from duckietown_utils.type_checks import dt_check_isinstance
from duckietown_utils.yaml_wrap import interpret_yaml_file, look_everywhere_for_config_files
from .algo_structures import EasyAlgoTest, EasyAlgoInstance, EasyAlgoFamily


def get_easy_algo_db():
    if EasyAlgoDB._singleton is None:
        EasyAlgoDB._singleton = get_cached('EasyAlgoDB', EasyAlgoDB)
    return EasyAlgoDB._singleton

class EasyAlgoDB():
    _singleton = None 
    
    pattern = '*.easy_algo_family.yaml'
    
    def __init__(self):
        self.family_name2config = load_family_config()
        
def load_family_config():
    """
        # now, for each family, we look for tests, which have name
        #  
        #     ![ID].![family_name]_test.yaml
        #
        # and configuration files, which are:
        #
        #     ![ID].![family_name]_test.yaml
        #
        # They both have the format:
        #   
    """
    family_name2config = {}
    configs = look_everywhere_for_config_files(EasyAlgoDB.pattern)
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
            dt_check_isinstance('parameters', parameters, dict) 

            return EasyAlgoTest(family_name=c.family_name, test_name=test_name,
                                description=description, filename=filename,
                                constructor=constructor, parameters=parameters)
            
        def interpret_instance_spec(filename, data):
            basename = os.path.basename(filename)
            instance_name = id_from_basename_pattern(basename, c.instances_pattern)
            
            description = data.pop('description')
            dt_check_isinstance('description', description, str) 

            constructor = data.pop('constructor')
            dt_check_isinstance('constructor', constructor, str) 

            parameters = data.pop('parameters')
            dt_check_isinstance('parameters', parameters, dict) 

            return EasyAlgoInstance(family_name=c.family_name, instance_name=instance_name,
                                    description=description, filename=filename,
                                    constructor=constructor, parameters=parameters)
            
        tests = {}
        
        _ = look_everywhere_for_config_files(c.tests_pattern)
        for filename, contents in _.items():
            t = interpret_yaml_file(filename, contents, interpret_test_spec)
            if t.test_name in tests:
                one = tests[t.test_name].filename
                two = t.filename
                msg = 'Repeated filename:\n%s\n%s' % (one, two)
                raise DTConfigException(msg)
            tests[t.test_name] = t
            
        instances = {}
        _ = look_everywhere_for_config_files(c.instances_pattern)
        for filename, contents in _.items():
            i = interpret_yaml_file(filename, contents, interpret_instance_spec)
            if i.instance_name in instances:
                one = instances[i.instance_name].filename
                two = i.filename
                msg = 'Repeated filename:\n%s\n%s' % (one, two)
                raise DTConfigException(msg)
            instances[i.instance_name] = i
        
        c = c._replace(tests=tests, instances=instances)
        family_name2config[c.family_name] = c
        
    return family_name2config
    
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
                            description=description)
    
    



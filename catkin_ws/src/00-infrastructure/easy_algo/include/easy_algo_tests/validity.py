from comptests.registrar import run_module_tests, comptest

from duckietown_utils.disk_hierarchy import dir_from_data
from easy_algo.algo_db import EasyAlgoDB
from easy_algo.formatting import format_db


@comptest 
def test_validity1():
    data="""
"F.easy_algo_family.yaml": | 
    description: desc
    interface: does.not_exist
"""
    d = dir_from_data(data)
    sources = [d]
    db = EasyAlgoDB(sources)
    
    # print format_db(db)
    
    family = db.get_family('F')
    assert family.valid == False

class MyAdderInterface(object):
    pass

class One(MyAdderInterface):
    pass

class Two(object):
    pass

@comptest 
def test_instance():
    data="""
"adder.easy_algo_family.yaml": | 
    description: desc
    interface: easy_algo_tests.validity.MyAdderInterface

"one.adder.yaml": |
    description: desc
    constructor: easy_algo_tests.validity.One
    parameters:

"not_sub.adder.yaml": |
    description: desc
    constructor: easy_algo_tests.validity.Two
    parameters:

"""
    d = dir_from_data(data)
    sources = [d]
    db = EasyAlgoDB(sources)
     
    print format_db(db)
    
    family = db.get_family('adder')
    assert family.valid == True
    
    one = db.create_instance('adder', 'one')
    
    assert type(one).__name__ == 'One'

    try:
        db.create_instance('not_found', 'does_not_exist')
        raise Exception()
    except Exception as e:
        assert 'not find' in str(e), e
    
    try:
        db.create_instance('adder', 'does_not_exist')
        raise Exception()
    except Exception as e:
        assert 'not find' in str(e), e
        
    try:
        db.create_instance('adder', 'not_sub')
        raise Exception()
    except Exception as e:
        assert 'MyAdderInterface' in str(e)


if __name__ == '__main__':
    run_module_tests()
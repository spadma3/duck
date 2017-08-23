from duckietown_utils.path_utils import display_filename
from duckietown_utils.text_utils import remove_table_field, format_table_plus  # @UnusedImport
from easy_algo.db.load_db import get_easy_algo_db, EasyAlgoFamily
from ruamel import yaml
from duckietown_utils.text_utils import indent

__all__ = ['summary']

def summary():
    db = get_easy_algo_db()
    S = '\n\n' 
    families = list(db.family_name2config.values())
    s = format_families(families)
    
    for family in families:
        s += S
        s += format_tests(family)
        s += S
        s += format_instances(family) 
        
    return s

def format_families(families):
    if not families:
        s = "No algorithm families found."
        return s
    else:
        s = "Found %d algorithm families:\n" % len(families)
        table = []
        table.append(['Family name', 
                  'description',
                  'interface',
                  'tests',  
                  'instances',
                  'where this family was defined'])
        for family in families:
            assert isinstance(family, EasyAlgoFamily)
            row = []        
            row.append(family.family_name)
            row.append(family.description)
            row.append(family.interface.replace('.','\n.'))
            _ = family.tests_pattern 
            if not family.tests:
                _ += '\n(no tests found)'
            else:
                _ += '\nFound %d tests:' % len(family.tests)
                for i, x in enumerate(sorted(family.tests)):
                    _ += "\n%d) %s" % (i+1, x)
                
            row.append(_)
            _ = family.instances_pattern 
            if not family.instances:
                _ += '\n(no instances found)'
            else:
                _ += '\nFound %d instances:' % len(family.instances)
                for i, x in enumerate(sorted(family.instances)):
                    _ += "\n%d) %s" % (i+1, x)
                
            row.append(_)
    
            row.append(display_filename(family.filename))
            table.append(row)
            
        s += indent(format_table_plus(table, colspacing=4), '| ')
        return s    
    
    
def format_tests(family):
    if not family.tests:
        s = ('No tests files found for family "%s" (pattern = %s).\n\n' % 
             (family.family_name, family.tests_pattern))
        return s
    else:
        s = 'Tests for family "%s"\n\n' % family.family_name
        table = []
        table.append(['Test name', 'description', 'constructor', 'parameters', 'filename'])
        for t in family.tests.values():
            row = []
            row.append(t.test_name)
            row.append(t.description)
            row.append(t.constructor)
            row.append(t.parameters)
            row.append(t.filename)
            table.append(row)
        s += format_table_plus(table, colspacing=4)    
        return s

def format_instances(family):
    if not family.instances:
        s = ('No instances files found for family "%s" (pattern = %s).\n\n' % 
             (family.family_name, family.instances_pattern))
        return s
    else:
        s = ('Found %d instances of algorithm family "%s":\n' % 
             (len(family.instances), family.family_name))
        table = []
        table.append(['Instance name', 'description', 'constructor', 'parameters', 'filename'])
        for _ in family.instances.values():
            row = []
            row.append(_.instance_name)
            row.append(_.description)
            row.append(_.constructor.replace('.','\n.'))
            row.append(yaml.dump(_.parameters))
            row.append(display_filename(_.filename))
            table.append(row)
        s += indent(format_table_plus(table, colspacing=4), '| ')
        return s
    

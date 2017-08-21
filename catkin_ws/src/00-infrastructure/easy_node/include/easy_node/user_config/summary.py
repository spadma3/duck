import termcolor

from duckietown_utils import col_logging  # @UnusedImport
from duckietown_utils.exceptions import wrap_script_entry_point
from duckietown_utils.path_utils import display_filename
from duckietown_utils.text_utils import truncate_string_right, format_table_plus
from easy_node.user_config.db import get_config_db
from easy_node.user_config.get_configuration_files import ConfigInfo


def user_config_summary():
    db = get_config_db()
    
    
    def apply_to_lines(f, x):
        return "\n".join(f(_) for _ in x.split('\n'))
        
    def red(s):
        red_ = lambda _:  termcolor.colored(_, 'red') 
        return apply_to_lines(red_, s)
    
    red = lambda x: x
     
    table = []
    
    table.append([ 'package name', 'node name', 'config_name', 'effective', 
                  'extends', 'valid', 'error', 'description', 'filename', ])
    for c in db.configs:
        assert isinstance(c, ConfigInfo)        
        d = truncate_string_right(c.description.replace('\n',' '), 40)
        date = c.date_effective.strftime('%Y-%m-%d')
        if c.valid is None:
            valid = '?'
            valid_error =''
        else:
            valid = 'yes' if c.valid else red('no')
            valid_error = '' if c.valid else red(c.error_if_invalid) 
        
        table.append([
            c.package_name, c.node_name, c.config_name,
            date , c.extends, valid, valid_error,
            d, display_filename(c.filename)
        ])
        
    def remove_field(f):
        i = table[0].index(f)
        for row in table:
            row.pop(i)
    
#     remove_field('filename')
        
    s = format_table_plus(table, colspacing=4)
    return s

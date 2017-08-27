import termcolor

from duckietown_utils import col_logging  # @UnusedImport
from duckietown_utils.friendly_path_imp import friendly_path
from duckietown_utils.text_utils import truncate_string_right, format_table_plus,\
    remove_table_field

from .db import get_config_db
from .get_configuration_files import ConfigInfo


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
            d, friendly_path(c.filename)
        ])
    
    remove_table_field(table, 'filename')
        
    s = format_table_plus(table, colspacing=4)
    return s

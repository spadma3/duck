#!/usr/bin/env python
from duckietown_utils import col_logging  # @UnusedImport
from duckietown_utils.exceptions import wrap_script_entry_point
from duckietown_utils.path_utils import display_filename
from duckietown_utils.text_utils import format_table,\
    truncate_string_left, truncate_string_right
from easy_node.get_configuration_files import get_all_configuration_files,\
    ConfigInfo
import os
from datetime import datetime


def summary():
    configs = get_all_configuration_files()
    table = []
    table.append([ 'package name', 'node name', 'config_name', 'effective', 'extends', 'description', 'filename'])
    for c in configs:
        assert isinstance(c, ConfigInfo)
        # ConfigInfo = namedtuple('ConfigInfo', 'filename package_name node_name config_name date_effective description values')
        
        d = truncate_string_right(c.description.replace('\n',' '), 40)
        date=  c.date_effective.strftime('%Y-%m-%d')
        table.append([
                    c.package_name, c.node_name, c.config_name, date , c.extends, d,
                    display_filename(c.filename)
                    ])
    
    print(format_table(table, colspacing=4))
        
    
    
if __name__ == '__main__':
    wrap_script_entry_point(summary)
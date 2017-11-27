from ruamel import yaml

from duckietown_utils import format_table_plus, remove_table_field,\
    make_row_red, friendly_path,  indent
from easy_logs.logs_db import get_easy_logs_db


def easy_logs_summary(query='*'):
    db = get_easy_logs_db()
    logs  = db.query(query) 
    s = format_logs(logs)
    return s
    
def format_logs(logs):
    if not logs:
        s = "No logs found."
        return s
    else:
        s = "Found %d logs.\n" % len(logs)
        table = []
        table.append(['#','Log name', 
                      'map',
                      'description',
                      'date',
                      'length',
                      'vehicle name',
                      'filename',
                      'valid',
                      'topics'])
        for i, (_, log) in enumerate(logs.items()):
            row = []      
            row.append(i)
            row.append(log.log_name)
            row.append(log.map_name)
            row.append(log.description)
            row.append(log.date)
            if log.length is not None:
                l = '%5.1f s' % log.length
            else:
                l = '(none)'
            row.append(l)
            row.append(log.vehicle)
            if log.filename is None:
                row.append('not local')
            else:
                row.append(friendly_path(log.filename))
            if log.valid:
                sr = 'Yes.'
            else:
                sr = log.error_if_invalid
            row.append(sr)
            if log.bag_info is not None:
                info = yaml.dump(log.bag_info['topics'])
            else:
                info = '(none)'
            if not log.valid:
                row = make_row_red(row)

            row.append(info)
            table.append(row)
            
        remove_table_field(table, 'filename')
        remove_table_field(table, 'topics')
        remove_table_field(table, 'description')
        remove_table_field(table, 'map')
        s += indent(format_table_plus(table, colspacing=4), '| ')
        return s    
    
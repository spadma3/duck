from duckietown_utils.instantiate_utils import indent
from duckietown_utils.path_utils import display_filename
from duckietown_utils.text_utils import format_table_plus, remove_table_field
from easy_logs.logs_db import load_all_logs


def easy_logs_summary():
    logs = load_all_logs()    
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
                      'filename'])
        for i, log in enumerate(logs):
            row = []      
            row.append(i)
            row.append(log.log_name)
            row.append(log.map_name)
            row.append(log.description)
            row.append(log.date)
            row.append(log.length)
            row.append(log.vehicle_name)
            row.append(display_filename(log.filename))
            table.append(row)
            
        remove_table_field(table, 'filename')
        s += indent(format_table_plus(table, colspacing=4), '| ')
        return s    
    
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.path_utils import display_filename
from duckietown_utils.text_utils import format_table_plus
from easy_algo.logs_db import load_all_logs


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
        table.append(['Log name', 
                      'map',
                      'description',
                      'vehicle name',
                      'filename'])
        for log in logs:
            
            row = []        
            row.append(log.log_name)
            row.append(log.map)
            row.append(log.description)
            row.append(log.vehicle_name)
            row.append(display_filename(log.vehicle_name))
            table.append(row)
            
        s += indent(format_table_plus(table, colspacing=4), '| ')
        return s    
    
from duckietown_utils.caching import get_cached
from duckietown_utils.yaml_wrap import look_everywhere_for_bag_files
from easy_logs.logs_structure import PhysicalLog


def get_easy_logs_db():
    if EasyLogsDB._singleton is None:
        EasyLogsDB._singleton = get_cached('EasyLogsDB', EasyLogsDB)
    return EasyLogsDB._singleton

class EasyLogsDB():
    _singleton = None 
     
    
    def __init__(self):
        self.logs = load_all_logs()
        
        

def load_all_logs():
    basename2filename = look_everywhere_for_bag_files()
    logs = []
    for basename, filename in basename2filename.items():
        log_name = basename
        filename = filename
        vehicle_name = None
        map_name = None
        description = None
        length = None
        date = None
        
        l = PhysicalLog(log_name=log_name, map_name=map_name, 
                        description=description, length=length,
                        date=date, 
                        filename=filename, vehicle_name=vehicle_name)
        logs.append(l)
    
    return logs
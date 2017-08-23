

def get_easy_logs_db():
    if EasyLogsDB._singleton is None:
        EasyLogsDB._singleton = get_cached('EasyLogsDB', EasyLogsDB)
    return EasyAlgoDB._singleton

class EasyLogsDB():
    _singleton = None 
    
    pattern = '*.easy_algo_family.yaml'
    
    def __init__(self):
        self.family_name2config = load_family_config()
        
        

def load_all_logs():
    
    logs = []
    
    return logs

def jobs_comptests(context):  
    from . import summary 
    from . import validity 
    from . import cli 
    
    from comptests.registrar import jobs_registrar_simple
    jobs_registrar_simple(context)
    

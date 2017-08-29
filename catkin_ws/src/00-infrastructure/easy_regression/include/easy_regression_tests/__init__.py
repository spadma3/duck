

def jobs_comptests(context):  
    
    from . import configuration
    from . import conditions  
    from . import binary  
    from . import references
    from . import evaluation
    from . import run_all
    
    from comptests.registrar import jobs_registrar_simple
    jobs_registrar_simple(context)
    

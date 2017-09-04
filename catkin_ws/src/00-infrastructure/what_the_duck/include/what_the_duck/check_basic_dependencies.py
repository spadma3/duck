from duckietown_utils import raise_desc

# Let's make sure we have the depedencies

try:
    import comptests  # @UnusedImport
except ImportError as e:
    msg = 'Comptests not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     pip install --user comptests'
    raise_desc(Exception, e, msg)
    
def check_procgraph():
    try:
        import procgraph  # @UnresolvedImport @UnusedImport
    except ImportError as e:
        msg = 'procgraph not installed.'
        msg += '\n\nTry the following:'
        msg += '\n\n     pip install --user procgraph'
        raise_desc(Exception, e, msg)
    

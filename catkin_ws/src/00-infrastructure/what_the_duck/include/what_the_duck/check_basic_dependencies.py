
# Let's make sure we have the depedencies

try:
    import comptests  # @UnusedImport
except:
    msg = 'Comptests not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     pip install --user comptests'
    raise Exception(msg)
    
    
try:
    import procgraph  # @UnresolvedImport @UnusedImport
except:
    msg = 'procgraph not installed.'
    msg += '\n\nTry the following:'
    msg += '\n\n     pip install --user procgraph'
    raise Exception(msg)
    

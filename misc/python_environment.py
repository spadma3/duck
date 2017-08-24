#!/usr/bin/env python
import sys,os,subprocess
print("version: %s" % str(sys.version_info))
print(" prefix: %r" % os.path.realpath(sys.prefix))

if False:
    for i, x in enumerate(sorted(sys.path)):
        print('sys.path[%d] = %s ' % (i, x))


prefixes = []
prefixes.append((sys.prefix, '%'))
prefixes.append((os.getcwd(), '.'))

def replace(where):
    where_real = str(os.path.realpath(where))
    for prefix, subst in prefixes:
        prefix = os.path.realpath(prefix)
        if where_real.startswith(prefix):
            return subst + where_real[len(prefix):]
    return where
    
for cmd in ['python', 'nosetests', 'pip', 'easy_install', 'ipython', 'patience', 'virtualenv']:
    output = subprocess.Popen(["which", cmd], stdout=subprocess.PIPE).communicate()[0].strip()
    where = output
    # where = subprocess.check_output('which', cmd]).strip()
    print("%12s: %s" % (cmd, replace(where)))

def show_version(module, version_func = lambda m: m.__version__):
    try:
        m = __import__(module, fromlist=['dummy'])
    except ImportError as e:
        print('%12s: not installed (%s)' % (module, e))
        return
    except Exception as e:
        print('%12s: ?? (%s)' % (module, e))
        return
    
    try:
        version = version_func(m)
    except:
        version = 'unknown'
    
    print("%12s: %10s in %s" % (module, version[:9], replace(m.__file__)))


which = sys.argv[1:]

if not which:
    which = []
    which += [('numpy')]
    which += [('scipy')]
    which += [('matplotlib')]
    which += [('tables')]

    which += [('contracts')]
    which += [('geometry')]
    which += [('compmake')]
    which += [('reprep')]

    which += [('procgraph')]
    which += [('playerc')]
    which += [('patience')]
    which += [('cjson')]
    which += [('PIL')]
    which += [('Image')]
    which += [('cv')]
    which += [('pyparsing')]
    which += [('decorator')]
    which += [('nose')]
    which += [('cairo')]

for x in which:
    show_version(x)
#show_version('flydra.version', lambda m: m.__version__) #lambda m: m.version.version)

if False:
    try: 
        import numpy;
        numpy.show_config()
    except:
        pass
    

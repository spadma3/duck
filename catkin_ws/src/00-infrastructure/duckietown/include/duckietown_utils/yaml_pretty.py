from ruamel import yaml as ruamel_yaml
 
def yaml_dump_pretty(ob):
    return ruamel_yaml.dump(ob, Dumper=ruamel_yaml.RoundTripDumper)


if True:
    from ruamel import yaml  # @UnresolvedImport
    # XXX: does not represent None as null, rather as '...\n'
    def yaml_load(s):
        if s.startswith('...'):
            return None
        l = yaml.load(s, Loader=yaml.RoundTripLoader)
        return remove_unicode(l)
    
    def yaml_dump(s):
        return yaml.dump(s, Dumper=yaml.RoundTripDumper, allow_unicode=False)
    
    
    def remove_unicode(x):
        
        if isinstance(x, unicode):
            return x.encode('utf8')

        if isinstance(x, dict):
            T = type(x)
            return T([(remove_unicode(k), remove_unicode(v)) for k,v in x.items()])

        if isinstance(x, list):
            T = type(x)
            return T([remove_unicode(_) for _ in x])
        
        return x
else:
    import yaml  # @Reimport
    def yaml_load(s):
        return yaml.load(s)
    
    def yaml_dump(s):
        return yaml.dump(s)
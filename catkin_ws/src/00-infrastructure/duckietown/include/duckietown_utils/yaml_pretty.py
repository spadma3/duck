from ruamel import yaml as ruamel_yaml
 
def yaml_dump_pretty(ob):
    return ruamel_yaml.dump(ob, Dumper=ruamel_yaml.RoundTripDumper)


if True:
    from ruamel import yaml  # @UnresolvedImport
    # XXX: does not represent None as null, rather as '...\n'
    def yaml_load(s):
        if s.startswith('...'):
            return None
        return yaml.load(s, Loader=yaml.RoundTripLoader)
    
    def yaml_dump(s):
        return yaml.dump(s, Dumper=yaml.RoundTripDumper)
else:
    import yaml  # @Reimport
    def yaml_load(s):
        return yaml.load(s)
    
    def yaml_dump(s):
        return yaml.dump(s)
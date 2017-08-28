from ruamel import yaml as ruamel_yaml
 
def yaml_dump_pretty(ob):
    return ruamel_yaml.dump(ob, Dumper=ruamel_yaml.RoundTripDumper)
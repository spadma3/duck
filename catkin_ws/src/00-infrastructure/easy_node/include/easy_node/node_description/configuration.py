from collections import namedtuple, OrderedDict
import os

from ruamel import yaml
from yaml.error import YAMLError

from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from duckietown_utils.instantiate_utils import import_name
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.system_cmd_imp import contract


# import yaml
__all__ = [
    'EasyNodeConfig',
    'load_configuration',
]

EasyNodeConfig = namedtuple('EasyNodeConfig', 'filename parameters subscriptions contracts publishers')
EasyNodeParameter = namedtuple('EasyNodeParameter', 'name desc type has_default default')
EasyNodeSubscription = namedtuple('EasyNodeSubscription', 'name desc type topic queue_size process latch')
EasyNodePublisher = namedtuple('EasyNodePublisher', 'name desc type topic queue_size latch')

PROCESS_THREADED = 'threaded'
PROCESS_SYNCHRONOUS = 'synchronous'
PROCESS_VALUES = [PROCESS_THREADED, PROCESS_SYNCHRONOUS]



# type = int, bool, float, or None (anything)
DEFAULT_NOT_GIVEN = 'default-not-given'

@contract(c1=EasyNodeConfig, c2=EasyNodeConfig, returns=EasyNodeConfig)
def merge_configuration(c1, c2):
    """ Merges two configurations. Values in c2 override the ones in c1 """
    parameters = OrderedDict()
    subscriptions = OrderedDict()
    contracts = OrderedDict()
    publishers = OrderedDict()
    for c in [c1, c2]:
        parameters.update(c.parameters)
        subscriptions.update(c.subscriptions)
        contracts.update(c.contracts)
        publishers.update(c.publishers)
    res = EasyNodeConfig(filename=c2.filename, # XXX
                         parameters=parameters, 
                         subscriptions=subscriptions, 
                         contracts=contracts,
                         publishers=publishers)
    return res
    
@contract(returns=EasyNodeConfig)
def load_configuration_package_node(package_name, node_type_name):
    path = get_ros_package_path(package_name)
    look_for = '%s.easy_node.yaml' % node_type_name
    found = locate_files(path, look_for)
    if not found:
        msg = 'Could not find EasyNode configuration %r.' % look_for
        raise DTConfigException(msg) # XXX
    
    fn = found[0]
    contents = open(fn).read()
    res = load_configuration(fn, contents)
    return res

@contract(returns=EasyNodeConfig)
def load_configuration(realpath, contents):
    # TODO: load "version" string
    try:
        try:
            data = yaml.load(contents)
        except YAMLError as e:
            msg = 'Could not read YAML file properly:' 
            raise_wrapped(DTConfigException, e, msg, compact=True)
        if not isinstance(data, dict):
            msg = 'Expected a dict, got %s.' % type(data).__name__
            raise DTConfigException(msg)
        try:
            parameters = data['parameters']
            subscriptions = data['subscriptions']
            publishers = data['publishers']
            contracts = data['contracts']
        except KeyError as e:
            key = e.args[0]
            msg = 'Invalid configuration: missing %r in\n %s' % (key, realpath)
            raise  DTConfigException(msg)
        
        parameters = load_configuration_parameters(parameters)
        subscriptions = load_configuration_subscriptions(subscriptions)
        contracts = load_configuration_contracts(contracts)
        publishers = load_configuration_publishers(publishers)
        
        return EasyNodeConfig(filename=realpath, parameters=parameters, contracts=contracts, 
                              subscriptions=subscriptions, publishers=publishers)
    except DTConfigException as e:
        msg = 'Invalid configuration at %s: ' % realpath
        raise_wrapped(DTConfigException, e, msg, compact=True)
    
def load_configuration_parameters(data):
    res = OrderedDict()
    for k, v in data.items():
        try:
            check_good_name(k)
            res[k] = load_configuration_parameter(k, v) 
        except DTConfigException as e:
            msg = 'Invalid parameter entry %r:' % k
            raise_wrapped(DTConfigException, e, msg, compact=True)
    return res

def load_configuration_subscriptions(data):
    res = OrderedDict()
    for k, v in data.items():
        try:
            check_good_name(k)
            res[k] = load_configuration_subscription(k, v)
        except DTConfigException as e:
            msg = 'Invalid subscription entry %r:' % k
            raise_wrapped(DTConfigException, e, msg, compact=True)
    return res

def load_configuration_publishers(data):
    res = OrderedDict()
    for k, v in data.items():
        try:
            check_good_name(k)
            res[k] = load_configuration_publisher(k, v)
        except DTConfigException as e:
            msg = 'Invalid publisher entry %r:' % k
            raise_wrapped(DTConfigException, e, msg, compact=True)
    return res

def preprocess_desc(d):
    if d is not None:
        return d.strip()
    
def load_configuration_parameter(name, data):
#     verbose:
#         desc: Whether the node is verbose or not.
#         type: bool
#         default: true
#     
    desc = data.pop('desc', None)
    desc = preprocess_desc(desc)
    type_ = data.pop('type')
    
    if 'default' in data:
        default = data.pop('default')
        has_default = True
    else:
        default = DEFAULT_NOT_GIVEN
        has_default = False
        
    if data:
        msg = 'Extra keys: %r' % data
        raise DTConfigException(msg)
    
    type2T = {
        'bool': bool,
        'str': str,
        'int': int,
        'float': float,
        'any': None,
    }
    
    if not type_ in type2T:
        raise NotImplementedError(type_)
    T = type2T[type_] 
    
    if has_default and default is not None and T is not None:
        default = T(default) 
    
    return EasyNodeParameter(name=name, desc=desc, type=T, 
                             has_default=has_default, default=default)

def check_good_name(k):
    # TODO
    pass

def message_class_from_string(s): 
    if not '/' in s:
        msg = ''
        msg += 'Invalid message name "%s".\n' % s
        msg += 'I expected that the name of the message is in the format "PACKAGE/MSG".\n '
        msg += 'E.g. "sensor_msgs/Joy" or "duckietown_msgs/BoolStamped".'
        raise DTConfigException(msg)

    # e.g. "std_msgs/Header"
    i = s.index('/')
    package = s[:i]
    name = s[i+1:]
    symbol = '%s.msg.%s' % (package, name)
    try:
        msgclass = import_name(symbol)
        return msgclass
    except:
        raise  
    
def load_configuration_subscription(name, data):
#      image:
#         desc: Image to read
#         topic: ~image
#         type: CompressedImage
#         queue_size: 1
    try:
        desc = data.pop('desc', None)
        desc = preprocess_desc(desc)
        latch = bool(data.pop('latch', False))
        topic = data.pop('topic')
        type_ = data.pop('type')
        queue_size = data.pop('queue_size', None)
        process = data.pop('process', PROCESS_SYNCHRONOUS)
        if not process in PROCESS_VALUES:
            msg = 'Invalid value of process %r not in %r.' % (process, PROCESS_VALUES)
            raise DTConfigException(msg)
        
    except KeyError as e:
        msg = 'Could not find field %r.' % e
        raise DTConfigException(msg)
    
    if data:
        msg = 'Extra keys: %r' % data
        raise DTConfigException(msg)
    T = message_class_from_string(type_)  
    
    return EasyNodeSubscription(name=name, desc=desc, topic=topic,
                                type=T, queue_size=queue_size, latch=latch, process=process)


def load_configuration_publisher(name, data):
    try:
        desc = data.pop('desc', None)
        desc = preprocess_desc(desc)
        latch = bool(data.pop('latch', False))
        topic = data.pop('topic')
        type_ = data.pop('type')
        queue_size = data.pop('queue_size', None)
    
    except KeyError as e:
        msg = 'Could not find field %r.' % e
        raise DTConfigException(msg)
    
    if data:
        msg = 'Extra keys: %r' % data
        raise DTConfigException(msg)
    
    T = message_class_from_string(type_)  
    
    return EasyNodePublisher(name=name, desc=desc,  topic=topic,
                                type=T, queue_size=queue_size, latch=latch)
    
def load_configuration_contracts(data):
    # TODO
    return {}
    
    
def load_configuration_for_nodes_in_package(package_name):
    """
        returns dict node_name -> config
    """
    suffix = '.easy_node.yaml'
    package_dir = get_ros_package_path(package_name)
    configs = locate_files(package_dir, '*' + suffix)
    res = {}
    for c in configs:
        node_name = os.path.basename(c).replace(suffix, '')
        res[node_name] = load_configuration_package_node(package_name, node_name)
    return res
        
    
    
    
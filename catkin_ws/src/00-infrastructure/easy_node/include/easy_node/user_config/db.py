from duckietown_utils import logger
from duckietown_utils.constants import get_list_of_packages_in_catkin_ws
from easy_node.node_description.configuration import load_configuration_for_nodes_in_package, EasyNodeConfig
from easy_node.user_config.get_configuration_files import get_all_configuration_files


class ValidationError(Exception):
    pass


class ConfigDB():
    def __init__(self):
        # Load all configuration
        logger.debug('Reading configuration files...')
        self.configs = get_all_configuration_files()
        self.package2nodes = {}
        
        logger.error('approximation')
        packages = get_list_of_packages_in_catkin_ws()
        packages = ['line_detector2'] 
        logger.debug('Reading %d packages configuration...' % len(packages))
        for p in packages:
            self.package2nodes[p] = load_configuration_for_nodes_in_package(p)

    def validate(self):
        logger.debug('Validating configuration...')
        for i, c in enumerate(self.configs):
            try:
                self.validate_file(c)
                c = c._replace(valid=True)
            except ValidationError as e:
                c = c._replace(valid=False)
                c = c._replace(error_if_invalid=str(e))
            self.configs[i] = c

    def validate_file(self, c):
        # first, check that indeed we have a package by that name
        if not c.package_name in self.package2nodes:
            msg = 'Invalid package "%s".' % c.package_name
            raise ValidationError(msg)
        # check that there is a node by that name
        if not c.node_name in self.package2nodes[c.package_name]:
            msg = 'No node "%s" in package "%s". ' % (c.node_name, c.package_name)
            raise ValidationError(msg)
        # check that all the extends exist
        for cn in c.extends:
            if not self.config_exists(c.package_name, c.node_name, cn):
                msg = 'Referenced config %s/%s/%s does not exist. ' % (c.package_name, c.node_name, cn)
                raise ValidationError(msg)
        # Finally, check that the values correspond to values that we have
        # in the node configuration
        node_config = self.package2nodes[c.package_name][c.node_name]
        assert isinstance(node_config, EasyNodeConfig)
        known = node_config.parameters
        for k in c.values:
            if k not in known:
                msg = 'The parameter "%s" is not known.\nKnown: %s.' % (k, sorted(known))
                raise ValidationError(msg) 
        
    


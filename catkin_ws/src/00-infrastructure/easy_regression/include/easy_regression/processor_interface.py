from abc import ABCMeta, abstractmethod

__all__ = [
    'ProcessorInterface',
]

class ProcessorInterface(object):
    
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def process_log(self, bag_in, bag_out):
        pass
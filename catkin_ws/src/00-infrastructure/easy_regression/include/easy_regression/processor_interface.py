from abc import ABCMeta, abstractmethod

class ProcessorInterface(object):
    
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def process_log(self, bag_in, bag_out):
        pass
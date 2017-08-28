from easy_regression.processor_interface import ProcessorInterface

class IdentityProcessor(ProcessorInterface):
    
    def process_log(self, bag_in, bag_out):
        for topic, msg, _t in bag_in.read_messages():
            bag_out.write(topic, msg)
            
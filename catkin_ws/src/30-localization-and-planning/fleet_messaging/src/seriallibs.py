# Import modules

import metric_pb2

def serialize(filename, tofile = 0):
    my_metric = metric_pb2.Metric()

    my_metric.name = 'sys.cpu'
    my_metric.type = 'gauge'
    my_metric.value = 99.9

    return my_metric.SerializeToString()

# deserialize from variable
def deserialize(msg):
    read_metric = metric_pb2.Metric()

    return read_metric.ParseFromString(msg)

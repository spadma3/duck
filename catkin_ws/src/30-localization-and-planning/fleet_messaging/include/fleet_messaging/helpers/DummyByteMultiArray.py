class DummyMultiArrayDimension(object):
    def __init__(self, label, size, stride):
        self.label = label
        self.size = size
        self.stride = stride


class DummyMultiArrayLayout(object):
    def __init__(self, data_offset):
        self.dim = []
        self.data_offset = data_offset

    def add(self, dim):
        self.dim.append(dim)


class DummyByteMultiArray(object):
    def __init__(self, layout, data):
        self.layout = layout
        self.data = data

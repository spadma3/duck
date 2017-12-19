"""
This file contains helper classes needed for the serialization and parsing of
messages sent between duckiebots.
"""


class MultiArrayDimension(object):
    """
    Dummy class analogous to MultiArrayDimension ROS Message.
    """
    def __init__(self, label, size, stride):
        """
        Constructor.
        Inputs: class members
        """
        self.label = label
        self.size = size
        self.stride = stride

    def get_members(self):
        """
        Get the class members.
        Outputs: Class members
        """
        return self.label, self.size, self.stride

    def set_members(self, label, size, stride):
        """
        Set the class members.
        Inputs: Class members
        """
        self.label = label
        self.size = size
        self.stride = stride

    def print_members(self):
        """
        Print the members of the class
        """
        print "label: {}".format(self.label)
        print "size: {}".format(self.size)
        print "stride: {}".format(self.stride)


class MultiArrayLayout(object):
    """
    Dummy class analogous to MultiArrayLayout ROS Message.
    """
    def __init__(self, dim, data_offset):
        """
        Constructor.
        Inputs: class members
        """
        self.dim = dim
        self.data_offset = data_offset

    def get_members(self):
        """
        Get the class members.
        Outputs: Class members
        """
        return self.dim, self.data_offset

    def set_members(self, dim, data_offset):
        """
        Set the class members.
        Inputs: Class members
        """
        self.dim = dim
        self.data_offset = data_offset

    def print_members(self):
        """
        Print the members of the class
        """
        for dim_element in self.dim:
            dim_element.print_members()
        print "data_offset: {}".format(self.data_offset)


class ByteMultiArray(object):
    """
    Dummy class analogous to ByteMultiArray ROS Message.
    """
    def __init__(self, layout, data):
        """
        Constructor.
        Inputs: class members
        """
        self.layout = layout
        self.data = data

    def get_members(self):
        """
        Get the class members.
        Outputs: Class members
        """
        return self.layout, self.data

    def set_members(self, layout, data):
        """
        Set the class members.
        Inputs: Class members
        """
        self.layout = layout
        self.data = data

    def print_members(self):
        """
        Print the members of the class
        """
        self.layout.print_members()
        print "data: {}".format(self.data)

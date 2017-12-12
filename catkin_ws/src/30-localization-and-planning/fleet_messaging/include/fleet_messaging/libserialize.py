"""
This file contains the serializer library for the fleet communication project.
"""


# Import modules
from proto.Any_pb2            import Any
from proto.ByteMultiArray_pb2 import ByteMultiArray
from copy                     import copy


def serialize_bma(ros_msg):
    """
    This function is used to serialize the ByteMultiArray data.
    Inputs:
    - ros_msg: ROS message to be serialized
    Outputs:
    - bma_data: serialized data (see ByteMultiArray.proto)
    """
    # Create the Serialization objects
    bma = ByteMultiArray()

    # Loop through the dimensions in the layout
    for dim in ros_msg.layout.dim:
        ma_dim = bma.layout.dim.add()
        ma_dim.label = dim.label
        ma_dim.size = dim.size
        ma_dim.stride = dim.stride

    # Set the data offset
    bma.layout.data_offset = ros_msg.layout.data_offset

    # Set the data
    bma.data = ros_msg.data

    bma_data = bma.SerializeToString()
    return bma_data

def serialize_any(name, bma_data):
    """
    This function is used to serialize the Any data.
    Inputs:
    - name:     name of the serialization (needed for Any.proto)
    - bma_data: serialized data (see ByteMultiArray.proto)
    Outputs:
    - any_data: serialized data (see Any.proto)
    """
    # Create the Serialization object
    any_proto = Any()

    # Set the data
    any_proto.type_url = name
    any_proto.value = bma_data

    # Serialize
    any_data = any_proto.SerializeToString()

    return any_data

def serialize(name, ros_msg):
    """
    This function is used to serialize the data using a two step
    serialization.
    Inputs:
    - name:    name of the serialization (needed for Any.proto)
    - ros_msg: ROS message to be serialized
    Outputs:
    - any_data: serialized data (byte string, resulting from Any.proto)
    """
    # Perform the seros_msgrialization
    bma_data = serialize_bma(ros_msg)
    any_data = serialize_any(name, bma_data)

    return any_data

def parse_any(data):
    """
    This function is used to parse the Any data.
    Inputs:
    - data: serialized data (byte string, resulting from Any.proto)
    Outputs:
    - name:     name of the serialization (needed for Any.proto)
    - bma_data: ByteMultiArray data (see ByteMultiArray.proto)
    """
    # Instantiate the proto object
    any_proto = Any()

    # Parse the data
    any_proto.ParseFromString(data)
    name = any_proto.type_url
    bma_data = any_proto.value

    return name, bma_data

def parse_bma(bma_data, mad, ros_msg):
    """
    This function is used to parse the ByteMultiArray data.
    Inputs:
    - bma_data: ByteMultiArray data (see ByteMultiArray.proto)
    - mad:      Empty MultiArrayDimension object for filling of the ROS message
    - ros_msg:  Empty ROS message
    Outputs:
    - ros_msg: ROS message ByteMultiArray
    """
    # Instantiate the proto object
    bma = ByteMultiArray()

    # Parse the data
    bma.ParseFromString(bma_data)

    # Populate the ROS message
    for dim in bma.layout.dim:
        mad.label = dim.label
        mad.size = dim.size
        mad.stride = dim.stride
        ros_msg.layout.dim.append(copy(mad))
    ros_msg.layout.data_offset = bma.layout.data_offset
    ros_msg.data = bma.data

    return ros_msg

def parse(data, mad, ros_msg):
    """
    This function is used to parse the data using a two step serialization.
    Inputs:
    - data:    serialized data (byte string, resulting from Any.proto)
    - mad:     Empty MultiArrayDimension object for filling of the ROS message
    - ros_msg: Empty ROS message
    Outputs:
    - name:    name of the serialization (needed for Any.proto)
    - ros_msg: ROS message with the parsed data
    """
    name, bma_data = parse_any(data)
    ros_msg = parse_bma(bma_data, mad, ros_msg)
    return name, ros_msg

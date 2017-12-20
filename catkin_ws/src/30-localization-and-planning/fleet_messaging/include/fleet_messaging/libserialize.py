"""
This file contains the serializer library for the fleet communication project.
"""


# Import modules
from proto.ByteMultiArray_pb2 import ByteMultiArray
import helpers.helpers as helpers


def serialize(ros_msg):
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
    bma.data = str(ros_msg.data)
    bma_data = bma.SerializeToString()
    
    return bma_data

def parse(bma_data):
    """
    This function is used to parse the ByteMultiArray data.
    Inputs:
    - bma_data: ByteMultiArray data (see ByteMultiArray.proto)
    - ros_msg:  Empty ROS message
    Outputs:
    - ros_msg: ROS message ByteMultiArray
    """
    # Instantiate the proto object
    bma = ByteMultiArray()

    # Parse the data
    bma.ParseFromString(bma_data)

    # Populate the ROS message
    dim_tmp = []
    for dim in bma.layout.dim:
        mad = helpers.MultiArrayDimension(dim.label, dim.size, dim.stride)
        dim_tmp.append(mad)
    layout_tmp = helpers.MultiArrayLayout(dim_tmp, bma.layout.data_offset)
    ros_msg = helpers.ByteMultiArray(layout_tmp, bma.data)

    return ros_msg

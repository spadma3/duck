from struct import pack, unpack, calcsize


class LocalizationMessageSerializer:
    """
    Serializes and deserializes the localization message to be sent over the network.
    """
    @staticmethod
    def serialize(robotName, tile):
        serialized_name = NameSerializer.serialize(robotName)
        serialized_tile = IntegerSerializer.serialize(tile)

        return bytearray(serialized_tile + serialized_name)

    @staticmethod
    def deserialize(bytes):
        """
        :return: Returns a 5-tuple: (robotName, x, y, z, rot)
        """
        tile_size = IntegerSerializer.size()
        tile = IntegerSerializer.deserialize(bytes[0:tile_size])
        name = NameSerializer.deserialize(bytes[tile_size:])
        return name, tile



class InstructionMessageSerializer:
    """
    Serializes and deserializes the instruction message to be sent over the network.
    """
    @staticmethod
    def serialize(robotName, targetNode, command):
        serializedName = NameSerializer.serialize(robotName)
        serializedTargetNode = IntegerSerializer.serialize(targetNode)
        serializedCommand = IntegerSerializer.serialize(command)

        return bytearray(serializedTargetNode + serializedCommand + serializedName)

    @staticmethod
    def deserialize(bytes):
        """
        :return: Returns a 3-tuple: (robotName, targetNode, command)
        """
        integerLength = IntegerSerializer.size()
        targetNode = IntegerSerializer.deserialize(bytes[0:integerLength])
        command = IntegerSerializer.deserialize(bytes[integerLength:2*integerLength])
        name = NameSerializer.deserialize(bytes[2*integerLength:])

        return name, targetNode, command


class NameSerializer:
    """
    Provides functionality to serialize and deserialize the robot name.
    """
    @staticmethod
    def serialize(name):
        return name

    @staticmethod
    def deserialize(bytes):
        return bytes

    @staticmethod
    def size(name):
        return len(name)

class TransformationSerializer(object):
    """
    Provides functionality to serialize and deserialize a the transformation of a duckiebot
    """
    serialization_format = '!ffff'

    @staticmethod
    def serialize(x, y, z, rot):
        """
        Serializes the position (x, y, z) and the rotation in binary format.
        """
        return pack(TransformationSerializer.serialization_format, x, y, z, rot)

    @staticmethod
    def deserialize(bytes):
        """
        Deserializes an array of bytes (represented as a string) back into the 4 values x, y, z
        and rotation.
        The function returns a 4-tuple: (x, y, z, rot)
        """
        return unpack(TransformationSerializer.serialization_format, bytes)

    @staticmethod
    def size():
        return calcsize(TransformationSerializer.serialization_format)


class IntegerSerializer:
    @staticmethod
    def serialize(number):
        return pack('!i', number)

    @staticmethod
    def deserialize(bytes):
        return unpack('!i', bytes)[0]

    @staticmethod
    def size():
        return calcsize('!i')


from struct import pack, unpack, calcsize


class LocalizationMessageSerializer:
    """
    Serializes and deserializes the localization message to be sent over the network.
    """
    @staticmethod
    def serialize(robotName, tile, route):
        serialized_name = NameSerializer.serialize(robotName)
        serialized_tile = IntegerSerializer.serialize(tile)
        serialized_route = StringListSerializer.serialize(route)

        return bytearray(serialized_tile + serialized_route + serialized_name )

    @staticmethod
    def deserialize(bytes):
        """
        :return: Returns a 3-tuple: (robotName, tile, route)
        """
        integer_size = IntegerSerializer.size()
        tile = IntegerSerializer.deserialize(bytes[0:integer_size])

        # Deserialize the list
        route_length = IntegerSerializer.deserialize(bytes[integer_size:2*integer_size])
        route_start = 2 * integer_size
        route_end = route_start + route_length
        route = StringListSerializer.deserialize(bytes[route_start:route_end])

        name = NameSerializer.deserialize(bytes[route_end:])

        return name, tile, route



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
        return StringSerializer.serialize(name)

    @staticmethod
    def deserialize(bytes):
        return StringSerializer.deserialize(bytes)

    @staticmethod
    def size(name):
        return StringSerializer.size(name)


class StringSerializer:
    """
    Provides functionality to serialize and deserialize the strings
    """
    @staticmethod
    def serialize(string):
        return string

    @staticmethod
    def deserialize(bytes):
        return bytes

    @staticmethod
    def size(string):
        return len(string)

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


class StringListSerializer:
    """
    Serializer for a list of strings.
    """
    SEPARATOR = ","

    @staticmethod
    def serialize(string_list):
        long_string = StringListSerializer.SEPARATOR.join(string_list)
        length = StringSerializer.size(long_string)
        return IntegerSerializer.serialize(length) + StringSerializer.serialize(long_string)

    @staticmethod
    def deserialize(bytes):
        long_string = StringSerializer.deserialize(bytes)
        return long_string.split(StringListSerializer.SEPARATOR)

    @staticmethod
    def size(string_list):
        long_string = StringListSerializer.SEPARATOR.join(string_list)
        return StringSerializer.size(long_string) + IntegerSerializer.size()



class IntegerListSerializer:
    """
    A list is serialized by first serializing the number of elements that follow.
    """
    @staticmethod
    def serialize(integer_list):
        data = ""
        data += IntegerSerializer.serialize(len(integer_list))

        for i in integer_list:
            data += IntegerSerializer.serialize(i)

        return data

    @staticmethod
    def deserialize(bytes, length):
        size = IntegerSerializer.size()
        integer_list = []
        for i in range(length):
            entry_data = bytes[i*size:(i+1)*size]
            integer_list.append(IntegerSerializer.deserialize(entry_data))

        return integer_list


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


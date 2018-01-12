#!/usr/bin/env python

import unittest

from message_serialization import *

class TestMessageSerialization(unittest.TestCase):

    def test_name_serialization(self):
        name = 'test'
        serializedName = NameSerializer.serialize(name)
        self.assertEqual(name, NameSerializer.deserialize(serializedName))

    def test_transformation_serializatio(self):
        x = 5
        y = 7
        z = 10
        rot = 90
        serializedData = TransformationSerializer.serialize(x, y, z, rot)

        new_x, new_y, new_z, new_rot = TransformationSerializer.deserialize(serializedData)
        self.assertEqual(x, new_x)
        self.assertEqual(y, new_y)
        self.assertEqual(z, new_z)
        self.assertEqual(rot, new_rot)

    def test_instruction_message_serialization(self):
        robotName = 'harpy'
        targetNode = 4
        command = 17
        serializedMessage = InstructionMessageSerializer.serialize(robotName, targetNode, command)
        new_name, new_targetNode, new_command = InstructionMessageSerializer.deserialize(serializedMessage)
        self.assertEqual(robotName, new_name)
        self.assertEqual(targetNode, new_targetNode)
        self.assertEqual(command, new_command)

    def test_localization_message_serialization(self):
        tile = 42
        name = 'harpy'
        route = ["5", "node 6", "3", "2"]
        serialized_message = LocalizationMessageSerializer.serialize(name, tile, route)
        new_name, new_tile, new_route = LocalizationMessageSerializer.deserialize(serialized_message)

        self.assertEqual(tile, new_tile)
        self.assertEqual(name, new_name)
        self.assertEqual(route, new_route)

    def test_list_serialization(self):
        integer_list = [1, 2, 3]
        serialized = IntegerListSerializer.serialize(integer_list)
        length = IntegerSerializer.deserialize(serialized[0:IntegerSerializer.size()])
        self.assertEqual(length, 3)

        deserialized_list = IntegerListSerializer.deserialize(serialized[IntegerSerializer.size():], length)
        self.assertEqual(integer_list, deserialized_list)

    def test_string_list_serialization(self):
        string_list = ["this", "is", "a", "test"]
        serialized = StringListSerializer.serialize(string_list)
        length = IntegerSerializer.deserialize(serialized[0:IntegerSerializer.size()])
        self.assertEqual(length, 14)

        deserialized_list = StringListSerializer.deserialize(serialized[IntegerSerializer.size():])
        self.assertEqual(string_list, deserialized_list)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('fleet_planning', 'test_message_serialization', TestMessageSerialization)
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
        serialized_message = LocalizationMessageSerializer.serialize(name, tile)
        new_name, new_tile = LocalizationMessageSerializer.deserialize(serialized_message)

        self.assertEqual(tile, new_tile)
        self.assertEqual(name, new_name)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('fleet_planning', 'test_message_serialization', TestMessageSerialization)
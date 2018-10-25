"""
Unit tests for the Task

@author: Mengers
"""

import unittest
from delegation_components.task import Task


class TaskTest(unittest.TestCase):
    """
    Unit tests for the Task
    """

    def setUp(self):
        self.task = Task(auction_id=1, auctioneer_name="test", goal_name="goal", depth=4)

    def test_getter(self):
        """
        Tests all properties
        """

        self.assertEqual(self.task.auction_id, 1)
        self.assertEqual(self.task.auctioneer_name, "test")
        self.assertEqual(self.task.goal_name, "goal")
        self.assertEqual(self.task.depth, 4)
        self.assertEqual(self.task.employers, [])
        self.task = Task(auction_id=1, auctioneer_name="test", goal_name="goal", depth=4, employers=[1, 2])
        self.assertEqual(self.task.auction_id, 1)
        self.assertEqual(self.task.auctioneer_name, "test")
        self.assertEqual(self.task.goal_name, "goal")
        self.assertEqual(self.task.depth, 4)
        self.assertEqual(self.task.employers, [1, 2])

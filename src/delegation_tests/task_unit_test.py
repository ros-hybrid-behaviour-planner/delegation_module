import unittest

from delegation_components.task import Task


class TaskTest(unittest.TestCase):

    def setUp(self):
        self.task = Task(auction_id=1, auctioneer_name="test", goal_name="goal")

    def test_getter(self):
        self.assertEqual(self.task.get_auction_id(), 1)
        self.assertEqual(self.task.get_auctioneer_name(), "test")
        self.assertEqual(self.task.goal_name(), "goal")

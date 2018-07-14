import unittest

from delegation_components.task import Task, EmployerAdministration


class TaskTest(unittest.TestCase):

    def setUp(self):
        self.task = Task(auction_id=1, auctioneer_name="test", goal_name="goal")
        self.employer_admin = EmployerAdministration()
        self.test_incidence = dict()
        self.test_incidence["1"] = 1
        self.test_incidence["2"] = 2

    def test_getter(self):
        self.assertEqual(self.task.get_auction_id(), 1)
        self.assertEqual(self.task.get_auctioneer_name(), "test")
        self.assertEqual(self.task.goal_name(), "goal")

    def test_task_employer_incidence(self):
        self.assertEqual(self.task.employer_incidence, {})
        self.task = Task(auction_id=1, auctioneer_name="test", goal_name="goal", employer_incidence=self.test_incidence)
        self.assertEqual(self.test_incidence, self.task.employer_incidence)

    def test_employer_admin(self):
        self.task = Task(auction_id=1, auctioneer_name="test", goal_name="goal", employer_incidence=self.test_incidence)
        res = self.employer_admin.add_task(self.task)
        self.assertEqual(self.employer_admin.get_current_employer_incidence(), res)
        self.assertEqual(self.employer_admin.get_current_employer_incidence()["1"], 1)
        self.assertEqual(self.employer_admin.get_current_employer_incidence()["2"], 2)
        res = self.employer_admin.add_task(self.task)
        self.assertEqual(self.employer_admin.get_current_employer_incidence(), res)
        self.assertEqual(self.employer_admin.get_current_employer_incidence()["1"], 2)
        self.assertEqual(self.employer_admin.get_current_employer_incidence()["2"], 4)
        res = self.employer_admin.remove_task(self.task)
        self.assertEqual(self.employer_admin.get_current_employer_incidence(), res)
        self.assertEqual(self.employer_admin.get_current_employer_incidence()["1"], 1)
        self.assertEqual(self.employer_admin.get_current_employer_incidence()["2"], 2)
        res = self.employer_admin.remove_task(self.task)
        self.assertEqual(self.employer_admin.get_current_employer_incidence(), res)
        self.assertEqual(len(self.employer_admin.get_current_employer_incidence().keys()), 0)

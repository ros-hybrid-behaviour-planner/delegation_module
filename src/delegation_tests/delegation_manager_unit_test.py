import unittest
import rospy

from delegation_tests.passive_node import DelegationMockingNode, AlwaysWinningCostEvaluator, MockingGoalWrapper
from delegation_components.delegation_manager import DelegationManager
from delegation_components.task import Task
from delegation_components.delegation_errors import DelegationError


class DelegationManagerTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node(name="TestNode")
        self.mock_name = "mocker"
        self.mock_manager = "mock_manager"
        self.interface_id = 1
        self.mocker = DelegationMockingNode(name=self.mock_name, manager_name=self.mock_manager)
        self.mocking_cost_eval = AlwaysWinningCostEvaluator()

    def test_setter_getter(self):
        name = "uut"
        uut = DelegationManager(instance_name=name, max_tasks=1)

        self.assertEqual(uut.get_name(), name)
        curr_id = uut.get_new_auction_id()
        self.assertEqual(uut.get_new_auction_id(), curr_id + 1)

    def test_multiple_tasks(self):
        name = "uut"
        id1, id2 = 1, 2
        n1, n2 = "name1", "name2"
        g1, g2 = "goal1", "goal2"
        task1 = Task(auction_id=id1, auctioneer_name=n1, goal_name=g1)
        task2 = Task(auction_id=id2, auctioneer_name=n2, goal_name=g2)

        # Max Tasks < 0
        uut = DelegationManager(instance_name=name, max_tasks=-1)
        self.assertFalse(uut.check_possible_tasks())
        self.assertRaises(DelegationError, uut.add_task, task1)
        self.assertRaises(DelegationError, uut.get_task_by_goal_name, g1)

        # Max Tasks = 0
        uut = DelegationManager(instance_name=name, max_tasks=0)
        self.assertFalse(uut.check_possible_tasks())
        self.assertRaises(DelegationError, uut.add_task, task1)
        self.assertRaises(DelegationError, uut.get_task_by_goal_name, g1)

        # Max tasks = 1
        uut = DelegationManager(instance_name=name, max_tasks=1)
        self.assertTrue(uut.check_possible_tasks())
        uut.add_task(new_task=task1)
        self.assertFalse(uut.check_possible_tasks())
        self.assertRaises(DelegationError, uut.add_task, task2)
        self.assertEqual(uut.get_task_by_goal_name(goal_name=g1), task1)
        self.assertRaises(DelegationError, uut.get_task_by_goal_name, g2)

        # Max tasks > 1
        uut = DelegationManager(instance_name=name, max_tasks=2)
        self.assertTrue(uut.check_possible_tasks())
        uut.add_task(new_task=task1)
        self.assertTrue(uut.check_possible_tasks())
        uut.add_task(new_task=task2)
        self.assertEqual(uut.get_task_by_goal_name(goal_name=g1), task1)
        self.assertEqual(uut.get_task_by_goal_name(goal_name=g2), task2)
        self.assertFalse(uut.check_possible_tasks())

    def test_cfp_callback(self):
        name = "uut"
        manager = "uut_manager"
        uut = DelegationManager(instance_name=name, max_tasks=1)

        auction_id = 1
        goal_rep = "test_goal"

        # No Manager present
        self.mocker.send_cfp(auction_id=auction_id, goal_representation=goal_rep)
        rospy.sleep(1)
        self.assertFalse(self.mocker.got_pro)

        # "Manager" present, cost computable
        uut.set_cost_function_evaluator(self.mocking_cost_eval, interface_id=self.interface_id, manager_name=manager)
        self.mocker.send_cfp(auction_id=auction_id, goal_representation=goal_rep)
        rospy.sleep(1)
        self.assertTrue(self.mocker.got_pro)
        res = self.mocker.Pro_last
        self.assertEqual(res.name, name)
        self.assertEqual(res.auction_id, auction_id)
        self.assertEqual(res.value, 0)  # the evaluator will always compute a 0

        self.mocker.reset_messages()

        # Deleted Manager
        uut.remove_cost_function_evaluator()
        self.mocker.send_cfp(auction_id=auction_id, goal_representation=goal_rep)
        rospy.sleep(1)
        self.assertFalse(self.mocker.got_pro)

    def test_delegate(self):
        name = "uut"
        goal_name = "test_goal"
        uut = DelegationManager(instance_name=name, max_tasks=1)
        test_goal = MockingGoalWrapper(name=goal_name)
        # TODO






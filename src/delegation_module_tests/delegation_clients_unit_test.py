"""
Unit tests for DelegationClient

@author: Mengers
"""

from .test_utils import MockedDelegationManager, MockedCostEvaluator
from delegation_components.delegation_clients import DelegationClientBase
from delegation_components.delegation_manager import DelegationManager
import unittest


class TestClient(DelegationClientBase):
    """
    Just a not abstract version of the base to test all methods except
    delegate, which has to be implemented by the user
    """

    def delegate(self, goal_name):
        return

    def start_work_for_delegation(self, delegation_id):
        return

    def delegation_successful(self, delegation_id):
        return


class AdditionalTestDelegationManager(object):
    """
    This has no other methods of the DM on purpose
    """

    @property
    def name(self):
        return "TestManager"


class DelegationClientTest(unittest.TestCase):
    """
    Unit test for the DelegationClient
    """

    def setUp(self):
        self.dm = MockedDelegationManager()

    def test_construction(self):
        """
        Tests constructor
        """

        uut = TestClient()

        self.assertEqual(DelegationClientBase.instance_counter, 1)
        self.assertEqual(DelegationClientBase.instance_counter, uut.id)
        self.assertEqual(uut, DelegationClientBase.get_client(client_id=uut.id))

    def test_registering(self):
        """
        Tests DelegationManager registering
        """

        uut = TestClient()

        self.assertFalse(uut._manager_active)

        uut.register(delegation_manager=self.dm)
        self.assertEqual(self.dm.clients[0], uut.id)
        self.assertEqual(uut._delegation_manager, self.dm)
        self.assertTrue(uut._manager_active)
        self.assertTrue(uut.check_if_registered())

        uut.register(delegation_manager=AdditionalTestDelegationManager())
        self.assertEqual(uut._delegation_manager, self.dm)

    def test_unregistering(self):
        """
        Tests DelegationManager unregistering
        """

        uut = TestClient()

        uut.unregister()    # should not raise an exception

        uut.register(delegation_manager=self.dm)

        uut.unregister()
        self.assertFalse(uut.check_if_registered())
        self.assertIsNone(uut._delegation_manager)
        self.assertEqual(len(self.dm.clients), 0)

        uut.register(delegation_manager=self.dm)

        # should not do anything, but no exception
        TestClient.unregister_at([5, 6])

        TestClient.unregister_at([uut.id])
        self.assertFalse(uut.check_if_registered())
        self.assertIsNone(uut._delegation_manager)
        self.assertEqual(len(self.dm.clients), 0)

    def test_cost_evaluator(self):
        """
        Tests CostEvaluator adding
        """

        uut = TestClient()

        manager = "test_manager"
        cost_eval = MockedCostEvaluator(cost=1, possibility=True)
        uut.add_own_cost_evaluator(cost_evaluator=cost_eval, agent_name=manager)

        uut.register(delegation_manager=self.dm)
        uut.add_own_cost_evaluator(cost_evaluator=cost_eval, agent_name=manager)
        self.assertEqual(self.dm.cfe, cost_eval)
        self.assertEqual(self.dm.agent_name, manager)
        self.assertEqual(self.dm.client_id, uut.id)

    def test_steps(self):
        """
        Tests do_step
        """

        uut = TestClient()
        delegations = [2, 3, 5]

        uut.do_step()

        uut.register(self.dm)
        uut._active_delegations.extend(delegations)

        uut.do_step()
        self.assertEqual(self.dm.stepped, delegations)

    def test_goal_removal(self):
        """
        Tests goal removal
        """

        uut = TestClient()
        goal = "TestGoal"

        uut.notify_goal_removal(goal_name=goal)

        uut.register(delegation_manager=self.dm)

        uut.notify_goal_removal(goal_name=goal)
        self.assertEqual([True, goal], self.dm.task_ended)

    def test_delegation(self):
        """
        Tests delegate_goal_wrapper
        """

        uut = TestClient()
        goal = "TestGoal"
        own_cost = 5
        known_depth = 2

        self.assertRaises(RuntimeError, uut.delegate_goal_wrapper, goal)

        uut.register(delegation_manager=self.dm)

        del_id = uut.delegate_goal_wrapper(goal_wrapper=goal, own_cost=own_cost, known_depth=known_depth)
        self.assertEqual(uut._active_delegations[0], 1)
        self.assertEqual(del_id, 1)
        self.assertEqual(self.dm.goal_wrapper, goal)
        self.assertEqual(self.dm.own_cost, own_cost)
        self.assertEqual(self.dm.steps, DelegationManager.DEFAULT_AUCTION_STEPS)
        self.assertEqual(self.dm.client_id, uut.id)
        self.assertEqual(self.dm.known_depth, known_depth)

    def test_terminations(self):
        """
        Tests termination functions
        """

        uut = TestClient()
        delegations = [1, 2, 3]

        uut._active_delegations.extend(delegations)
        uut.terminate_delegation(delegation_id=1)
        uut.terminate_all_delegations()

        uut = TestClient()
        uut.register(delegation_manager=self.dm)
        uut._active_delegations.extend(delegations)
        uut.terminate_all_delegations()
        self.assertEqual(len(uut._active_delegations), 0)
        self.assertEqual(delegations, self.dm.terminated)

        self.dm.terminated = []
        uut._active_delegations.extend(delegations)
        uut.terminate_delegation(delegation_id=delegations[0])
        self.assertEqual([delegations[0]], self.dm.terminated)
        delegations.remove(delegations[0])
        self.assertEqual(uut._active_delegations, delegations)

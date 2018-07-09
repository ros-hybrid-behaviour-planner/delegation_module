

from delegation_tests.test_utils import MockedDelegationManager, MockedCostEvaluator
from delegation_components.delegation_clients import DelegationClientBase
import unittest


class TestClient(DelegationClientBase):
    """
    Just a not abstract version of the base to test all methods except
    delegate, which has to be implemented by the user
    """

    def delegate(self, goal_name):
        return


class AdditionalTestDelegationManager(object):
    """
    This has no other methods of the DM on purpose
    """

    def get_name(self):
        return "TestManager"


class DelegationClientTest(unittest.TestCase):

    def setUp(self):
        self.dm = MockedDelegationManager()

    def test_construction(self):
        uut = TestClient()

        self.assertEqual(DelegationClientBase.instance_counter, 1)
        self.assertEqual(DelegationClientBase.instance_counter, uut.id)
        self.assertEqual(uut, DelegationClientBase.get_client(client_id=uut.id))

    def test_registering(self):
        uut = TestClient()

        self.assertFalse(uut._active_manager)

        uut.register(delegation_manager=self.dm)
        self.assertEqual(self.dm.clients[0], uut.id)
        self.assertEqual(uut._delegation_manager, self.dm)
        self.assertTrue(uut._active_manager)
        self.assertTrue(uut.check_if_registered())

        uut.register(delegation_manager=AdditionalTestDelegationManager())
        self.assertEqual(uut._delegation_manager, self.dm)

    def test_unregistering(self):
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
        uut = TestClient()

        manager = "test_manager"
        costeval = MockedCostEvaluator(cost=1, possibility=True)
        uut.add_own_cost_evaluator(cost_evaluator=costeval, manager_name=manager)

        uut.register(delegation_manager=self.dm)
        uut.add_own_cost_evaluator(cost_evaluator=costeval, manager_name=manager)
        self.assertEqual(self.dm.cfe, costeval)
        self.assertEqual(self.dm.m_name, manager)
        self.assertEqual(self.dm.client_id, uut.id)

    def test_steps(self):
        uut = TestClient()
        delegations = [2, 3, 5]

        uut.do_step()

        uut.register(self.dm)
        uut._active_delegations.extend(delegations)

        uut.do_step()
        self.assertEqual(self.dm.stepped, delegations)

    def test_goal_removal(self):
        uut = TestClient()
        goal = "TestGoal"

        uut.notify_goal_removal(goal_name=goal)

        uut.register(delegation_manager=self.dm)

        uut.notify_goal_removal(goal_name=goal)
        self.assertEqual([True, goal], self.dm.task_ended)

    def test_delegation(self):
        uut = TestClient()
        goal = "Testgoal"

        self.assertRaises(RuntimeError, uut.delegate_goal_wrapper, goal)

        uut.register(delegation_manager=self.dm)

        del_id = uut.delegate_goal_wrapper(goal_wrapper=goal)
        self.assertEqual(uut._active_delegations[0], 1)
        self.assertEqual(del_id, 1)
        self.assertEqual(self.dm.goal_wrapper, goal)

    def test_terminations(self):
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
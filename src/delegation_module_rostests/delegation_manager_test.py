#! /usr/bin/env python2
"""
Unit tests for the DelegationManager

Needs a running ROSCORE!

@author: Mengers, hrabia
"""

import unittest
import rospy
import rostest
from delegation_module_tests.test_utils import MockedDelegationCommunicator, MockedCostEvaluator, MockedGoalWrapper, MockedClient
from delegation_components.delegation_manager import DelegationManager
from delegation_components.task import Task
from delegation_components.delegation_errors import DelegationError
from delegation_components.delegation import Delegation
from dynamic_reconfigure.client import Client as DynRecClient


class DelegationManagerTest(unittest.TestCase):
    """
    Unit tests for the DelegationManager
    """

    def __init__(self, *args, **kwargs):
        super(DelegationManagerTest, self).__init__(*args, **kwargs)
        rospy.init_node(name="DelegationManagerTest", log_level=rospy.DEBUG)

    def setUp(self):
        self.mocked_DM_name = "MockedDM"
        self.mocked_manager_name = "MockedManager"
        self.uut_name = "UUT"
        self.uut_mocked_manager_name = "UUTManager"
        self.mocked_client = MockedClient()
        self.mocked_client_id = self.mocked_client.id
        self.mocked_DM = MockedDelegationCommunicator(name=self.mocked_DM_name, manager_name=self.mocked_manager_name)
        self.mocked_cost_eval = MockedCostEvaluator(cost=0, possibility=True)
        self.standard_depth = 2
        self.standard_members = ["Member1", "Member2"]
        self.uut = self.new_uut()
        rospy.sleep(0.3)
        self.dynrecClient = DynRecClient("UUT/delegation_module_parameters")
        self.dynrecClient.update_configuration({"max_tasks": 1})

    def tearDown(self):
        self.mocked_DM.__del__()
        self.uut.__del__()

    def new_uut(self):
        """
        Deletes old Unit under Test, UUT, and creates a new one. Also resets
        messages and config

        :return: uut
        :rtype: DelegationManager
        """

        if hasattr(self, 'uut'):
            self.uut.__del__()
            self.uut = None
            rospy.sleep(0.5)
        uut = DelegationManager(name=self.uut_name)
        self.mocked_DM.reset_messages()
        self.uut = uut

        self.dynrecClient = None
        while not self.dynrecClient:
            try:
                self.dynrecClient = DynRecClient("/UUT/delegation_module_parameters", timeout=1)
            except rospy.ROSInterruptException:
                pass
            rospy.sleep(0.3)
            rospy.logwarn("NO DynRecClient created yet, waiting for dynamic_reconfigure services.")

        return uut

    def test_setter_getter(self):
        """
        Tests the properties
        """
        rospy.loginfo("")
        uut = self.new_uut()

        self.assertEqual(uut.name, self.uut_name)
        curr_id = uut.get_new_auction_id()
        self.assertEqual(uut.get_new_auction_id(), curr_id + 1)

    # TODO this test is running fine if executed manually from pycharm but makes problems from the commandline
    @unittest.SkipTest
    def test_multiple_tasks(self):
        """
        Tests adding tasks
        """

        rospy.loginfo("test_multiple_tasks")
        
        id1, id2 = 1, 2
        n1, n2 = "name1", "name2"
        g1, g2 = "goal1", "goal2"
        task1 = Task(auction_id=id1, auctioneer_name=n1, goal_name=g1, depth=self.standard_depth)
        task2 = Task(auction_id=id2, auctioneer_name=n2, goal_name=g2, depth=self.standard_depth)

        # Max Tasks = -1 (infinite)
        self.dynrecClient.update_configuration({"max_tasks":-1})
        rospy.sleep(0.3)
        uut = self.new_uut()
        rospy.sleep(1)
        self.assertTrue(uut.check_possible_tasks)
        uut.add_task(new_task=task2)
        uut.add_task(new_task=task1)
        self.assertTrue(uut.check_possible_tasks)

        # Max Tasks = 0
        self.dynrecClient.update_configuration({"max_tasks":0})
        rospy.sleep(0.3)
        uut = self.new_uut()
        self.assertFalse(uut.check_possible_tasks)
        self.assertRaises(DelegationError, uut.add_task, task1)
        self.assertRaises(LookupError, uut.get_task_by_goal_name, g1)

        # Max tasks = 1
        self.dynrecClient.update_configuration({"max_tasks":1})
        rospy.sleep(0.3)
        uut = self.new_uut()
        self.assertTrue(uut.check_possible_tasks)
        uut.add_task(new_task=task1)
        self.assertFalse(uut.check_possible_tasks)
        self.assertRaises(DelegationError, uut.add_task, task2)
        self.assertEqual(uut.get_task_by_goal_name(goal_name=g1), task1)
        self.assertRaises(LookupError, uut.get_task_by_goal_name, g2)

        # Max tasks > 1
        self.dynrecClient.update_configuration({"max_tasks":2})
        rospy.sleep(0.3)
        uut = self.new_uut()
        self.assertTrue(uut.check_possible_tasks)
        uut.add_task(new_task=task1)
        self.assertTrue(uut.check_possible_tasks)
        uut.add_task(new_task=task2)
        self.assertEqual(uut.get_task_by_goal_name(goal_name=g1), task1)
        self.assertEqual(uut.get_task_by_goal_name(goal_name=g2), task2)
        self.assertFalse(uut.check_possible_tasks)

    def test_cfp_callback(self):
        """
        Tests CFP callback
        """
        rospy.loginfo("test_cfp_callback")
        uut = self.new_uut()

        auction_id = 1
        goal_rep = "test_goal"
        depth = 2

        # No Manager present
        self.mocked_DM.send_cfp(auction_id=auction_id, goal_representation=goal_rep, depth=depth, delegation_members=self.standard_members)
        rospy.sleep(1)
        self.assertFalse(self.mocked_DM.got_pro)

        # "Manager" present, cost computable
        uut.set_cost_function_evaluator(self.mocked_cost_eval, client_id=self.mocked_client_id, agent_name=self.uut_mocked_manager_name)
        self.mocked_DM.send_cfp(auction_id=auction_id, goal_representation=goal_rep, depth=depth, delegation_members=self.standard_members)
        rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_pro)
        res = self.mocked_DM.Pro_last
        self.assertEqual(res.name, self.uut_name)
        self.assertEqual(res.auction_id, auction_id)
        self.assertEqual(res.value, 0)  # the evaluator will always compute a 0

        self.mocked_DM.reset_messages()

        # Deleted Manager
        uut.remove_cost_function_evaluator()
        self.mocked_DM.send_cfp(auction_id=auction_id, goal_representation=goal_rep, depth=depth, delegation_members=self.standard_members)
        rospy.sleep(1)
        self.assertFalse(self.mocked_DM.got_pro)

    def test_delegate(self):
        """
        Tests delegate
        """
        rospy.loginfo("test_delegate")
        uut = self.new_uut()
        goal_name = "test goal"
        test_goal = MockedGoalWrapper(name=goal_name)
        steps = 2

        rospy.sleep(1)  # ros msgs directly after startup are lost sometimes
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id)
        rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_cfp)
        cfp = self.mocked_DM.CFP_last
        self.assertEqual(cfp.goal_representation, goal_name)
        self.assertEqual(cfp.name, self.uut_name)
        self.assertEqual(cfp.auction_id, auction_id)

        delegation = uut.get_delegation(auction_id=auction_id)
        self.assertIsInstance(delegation, Delegation)
        self.assertTrue(delegation.state.is_waiting_for_proposals())
        self.assertEqual(delegation.auction_id, auction_id)
        self.assertEqual(delegation.client_id, self.mocked_client_id)

    def test_propose_callback(self):
        """
        Tests PROPOSE callback
        """
        rospy.loginfo("test_propose_callback")
        uut = self.new_uut()
        goal_name = "test_goal"
        test_goal = MockedGoalWrapper(name=goal_name)
        steps = 2
        proposed_value = 2

        rospy.sleep(1)  # ros msgs directly after startup are lost sometimes
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id)
        rospy.sleep(1)
        cfp = self.mocked_DM.CFP_last
        self.mocked_DM.send_propose(value=proposed_value, target_name=cfp.name, auction_id=cfp.auction_id)
        rospy.sleep(1)
        delegation = uut.get_delegation(auction_id=auction_id)
        proposal = delegation.get_best_proposal()
        self.assertEqual(proposal.name, self.mocked_DM_name)
        self.assertEqual(proposal.value, proposed_value)

    def test_ending_auctions(self):
        """
        Tests the Ending of auctions
        """
        rospy.loginfo("test_ending_auctions")
        uut = self.new_uut()
        goal_name = "test_goal"
        test_goal = MockedGoalWrapper(name=goal_name)
        test_depth = 3
        test_task = Task(auction_id=100, goal_name=goal_name, depth=test_depth, auctioneer_name="test")
        uut.set_cost_function_evaluator(client_id=1, cost_function_evaluator=self.mocked_cost_eval, agent_name=self.mocked_manager_name)
        uut.add_task(test_task)
        steps = 3
        proposed_value = 2

        # No proposals
        rospy.sleep(1)  # ros msgs directly after startup are lost sometimes
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id)
        rospy.sleep(1)
        self.mocked_DM.reset_messages()
        for i in range(steps-1):
            uut.do_step(delegation_ids=[auction_id])
            rospy.sleep(1)
            self.assertFalse(self.mocked_DM.got_cfp)
        uut.do_step(delegation_ids=[auction_id])
        rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_cfp)
        self.assertEqual(self.mocked_DM.CFP_last.auction_id, auction_id)
        uut.terminate(auction_id=auction_id)

        # a proposal, but not true anymore, not bidding anymore
        test_goal = MockedGoalWrapper(name=goal_name)
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id)
        rospy.sleep(1)
        self.mocked_DM.send_propose(value=proposed_value, target_name=self.uut_name, auction_id=auction_id)
        self.mocked_DM.reset_messages()
        self.mocked_DM.set_precom_response(acceptance=False, still_bidding=False, cost=proposed_value)
        for i in range(steps):
            uut.do_step(delegation_ids=[auction_id])
            rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_pre)
        self.assertTrue(self.mocked_DM.got_cfp)
        uut.terminate(auction_id=auction_id)

        # a proposal, but not true anymore, still bidding
        test_goal = MockedGoalWrapper(name=goal_name)
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id)
        rospy.sleep(1)
        self.mocked_DM.send_propose(value=proposed_value, target_name=self.uut_name, auction_id=auction_id)
        self.mocked_DM.reset_messages()
        proposed_value += 1
        self.mocked_DM.set_precom_response(acceptance=False, still_bidding=True, cost=proposed_value)
        for i in range(steps):
            uut.do_step(delegation_ids=[auction_id])
            rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_pre)
        self.assertTrue(self.mocked_DM.got_cfp)
        uut.terminate(auction_id=auction_id)

        # a proposal, but no answer to precommit
        test_goal = MockedGoalWrapper(name=goal_name)
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id)
        rospy.sleep(1)
        self.mocked_DM.send_propose(value=proposed_value, target_name=self.uut_name, auction_id=auction_id)
        self.mocked_DM.reset_messages()
        self.mocked_DM.stop_communication()
        for i in range(steps):
            uut.do_step(delegation_ids=[auction_id])
            rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_cfp)
        uut.terminate(auction_id=auction_id)
        self.mocked_DM.start_communication()

        # proposal and accepted precom
        test_goal = MockedGoalWrapper(name=goal_name)
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id)
        rospy.sleep(1)
        self.mocked_DM.send_propose(value=proposed_value, target_name=self.uut_name, auction_id=auction_id)
        self.mocked_DM.reset_messages()
        self.mocked_DM.set_precom_response(acceptance=True, still_bidding=True, cost=proposed_value)
        for i in range(steps):
            uut.do_step(delegation_ids=[auction_id])
            rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_pre)
        self.assertEqual(self.mocked_DM.Pre_last.depth, test_depth+1)
        self.assertTrue(test_goal.goal_is_created())
        self.assertEqual(test_goal.get_manager(), self.mocked_manager_name)
        uut.terminate(auction_id=auction_id)

        # own cost wins
        test_goal = MockedGoalWrapper(name=goal_name)
        own_cost = 2
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id, own_cost=own_cost)
        for i in range(steps):
            uut.do_step(delegation_ids=[auction_id])
            rospy.sleep(1)
        self.assertFalse(test_goal.goal_is_created())
        self.assertTrue(self.mocked_client.started_working)

    # TODO this test is running fine if executed manually from pycharm but makes problems from the commandline
    @unittest.SkipTest
    def test_terminate(self):
        """
        Tests terminate
        """
        rospy.loginfo("test_terminate")
        uut = self.new_uut()
        goal_name = "test_goal"
        test_goal = MockedGoalWrapper(name=goal_name)
        steps = 3
        proposed_value = 2

        # proposal and accepted precom
        auction_id = uut.delegate(goal_wrapper=test_goal, auction_steps=steps, client_id=self.mocked_client_id)
        rospy.sleep(1)
        self.mocked_DM.send_propose(value=proposed_value, target_name=self.uut_name, auction_id=auction_id)
        self.mocked_DM.reset_messages()
        self.mocked_DM.set_precom_response(acceptance=True, still_bidding=True, cost=proposed_value)
        for i in range(steps):
            uut.do_step(delegation_ids=[auction_id])
            rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_pre)
        self.assertTrue(test_goal.goal_is_created())
        self.assertEqual(test_goal.get_manager(), self.mocked_manager_name)

        uut.terminate(auction_id=auction_id)
        delegation = uut.get_delegation(auction_id=auction_id)
        self.assertFalse(test_goal.goal_is_created())
        self.assertTrue(delegation.state.is_finished())

    # TODO this test is running fine if executed manually from pycharm but makes problems from the commandline
    @unittest.SkipTest
    def test_precom_callback(self):
        """
        Tests PRECOM callback
        """
        rospy.loginfo("test_precom_callback")
        auction_id = 1
        goal_name = "test_goal"
        old_proposal = 3

        # cost not computable
        uut = self.new_uut()
        response = self.mocked_DM.send_precom(target_name=self.uut_name, auction_id=auction_id, proposal_value=old_proposal, goal_name=goal_name, goal_representation=goal_name, depth=self.standard_depth, delegation_members=self.standard_members)
        self.assertFalse(response.acceptance)
        self.assertFalse(response.still_biding)

        # max tasks = 0
        self.dynrecClient.update_configuration({"max_tasks": 0})
        rospy.sleep(0.3)
        uut = self.new_uut()
        uut.set_cost_function_evaluator(cost_function_evaluator=self.mocked_cost_eval, agent_name=self.uut_mocked_manager_name, client_id=self.mocked_client_id)
        response = self.mocked_DM.send_precom(target_name=self.uut_name, auction_id=auction_id, proposal_value=old_proposal, goal_name=goal_name, goal_representation=goal_name, depth=self.standard_depth, delegation_members=self.standard_members)
        self.assertFalse(response.acceptance)
        self.assertFalse(response.still_biding)

        # not possible anymore
        self.dynrecClient.update_configuration({"max_tasks": 1})
        rospy.sleep(0.3)
        uut = self.new_uut()
        uut.set_cost_function_evaluator(cost_function_evaluator=MockedCostEvaluator(cost=0, possibility=False), agent_name=self.uut_mocked_manager_name, client_id=self.mocked_client_id)
        response = self.mocked_DM.send_precom(target_name=self.uut_name, auction_id=auction_id, proposal_value=old_proposal, goal_name=goal_name, goal_representation=goal_name, depth=self.standard_depth, delegation_members=self.standard_members)
        self.assertFalse(response.acceptance)
        self.assertFalse(response.still_biding)

        # cost is worse now
        new_cost = old_proposal + 1
        self.dynrecClient.update_configuration({"max_tasks": 1})
        rospy.sleep(0.3)
        uut = self.new_uut()
        uut.set_cost_function_evaluator(cost_function_evaluator=MockedCostEvaluator(cost=new_cost, possibility=True), agent_name=self.uut_mocked_manager_name, client_id=self.mocked_client_id)
        response = self.mocked_DM.send_precom(target_name=self.uut_name, auction_id=auction_id, proposal_value=old_proposal, goal_name=goal_name, goal_representation=goal_name, depth=self.standard_depth, delegation_members=self.standard_members)
        self.assertFalse(response.acceptance)
        self.assertTrue(response.still_biding)
        self.assertEqual(response.new_proposal, new_cost)

        # cost is exactly the same
        self.dynrecClient.update_configuration({"max_tasks": -1})
        rospy.sleep(0.3)
        new_cost = old_proposal
        uut = self.new_uut()
        uut.set_cost_function_evaluator(cost_function_evaluator=MockedCostEvaluator(cost=new_cost, possibility=True), agent_name=self.uut_mocked_manager_name, client_id=self.mocked_client_id)
        response = self.mocked_DM.send_precom(target_name=self.uut_name, auction_id=auction_id, proposal_value=old_proposal, goal_name=goal_name, goal_representation=goal_name, depth=self.standard_depth, delegation_members=self.standard_members)
        self.assertTrue(response.acceptance)
        self.assertEqual(response.manager_name, self.uut_mocked_manager_name)

    def test_cost_function_adding_removing(self):
        """
        Tests CostEvaluator adding/removing
        """
        rospy.loginfo("test_cost_function_adding_removing")
        uut = self.new_uut()
        uut.set_cost_function_evaluator(cost_function_evaluator=self.mocked_cost_eval, agent_name=self.uut_mocked_manager_name, client_id=self.mocked_client_id)
        self.assertTrue(uut.cost_computable)
        uut.remove_cost_function_evaluator()
        self.assertFalse(uut.cost_computable)

    def test_fail_task(self):
        """
        Tests FAIL callback
        """
        rospy.loginfo("test_fail_task")
        uut = self.new_uut()
        auction_id = 1
        goal_name = "test goal"
        task = Task(auction_id=auction_id, auctioneer_name=self.mocked_DM_name, goal_name=goal_name, depth=self.standard_depth)

        uut.add_task(new_task=task)
        uut.fail_task(goal_name=goal_name)
        rospy.sleep(1)
        self.assertTrue(self.mocked_DM.got_fail)
        request = self.mocked_DM.Fail_last
        self.assertEqual(self.uut_name, request.name)
        self.assertEqual(request.auction_id, auction_id)

    def test_end_task(self):
        """
        Tests end task
        """
        rospy.loginfo("test_end_task")
        uut = self.new_uut()
        auction_id = 1
        goal_name = "test goal"
        task = Task(auction_id=auction_id, auctioneer_name=self.mocked_DM_name, goal_name=goal_name, depth=self.standard_depth)

        uut.add_task(new_task=task)
        uut.end_task(goal_name=goal_name)
        self.assertRaises(LookupError, uut.get_task_by_goal_name, goal_name)


if __name__ == '__main__':
    rostest.rosrun('delegation_module', 'test_delegation_manager_node', DelegationManagerTest)
    rospy.spin()

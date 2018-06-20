import unittest

from delegation_components.delegation import Delegation, Proposal
from delegation_components.delegation_errors import DelegationContractorError
from delegation_tests.test_utils import MockedGoalWrapper


class DelegationTest(unittest.TestCase):

    def setUp(self):
        self.goal_name = "test_goal"
        self.wrapper = MockedGoalWrapper(name=self.goal_name)

    def test_contractors(self):
        delegation = Delegation(goal_wrapper=self.wrapper, auction_id=1)
        self.assertRaises(DelegationContractorError, delegation.get_contractor)

        name = "contractor"
        delegation.set_contractor(name=name)
        self.assertTrue(delegation.state.is_delegated_running())
        self.assertEqual(delegation.get_contractor(), name)
        self.assertRaises(DelegationContractorError, delegation.set_contractor, name)

        delegation.remove_contractor()
        self.assertRaises(DelegationContractorError, delegation.get_contractor)

    def test_proposals(self):
        delegation = Delegation(goal_wrapper=self.wrapper, auction_id=1)
        self.assertRaises(LookupError, delegation.get_best_proposal)
        self.assertFalse(delegation.has_proposals())

        first = Proposal("test1", 3)
        delegation.add_proposal(first)
        self.assertEqual(delegation.get_best_proposal(), first)
        self.assertTrue(delegation.has_proposals())

        better = Proposal("test2", 1)
        delegation.add_proposal(better)
        self.assertEqual(delegation.get_best_proposal(), better)

        delegation.remove_proposal(better)
        self.assertEqual(delegation.get_best_proposal(), first)

        delegation.add_proposal(better)
        delegation.reset_proposals()
        self.assertRaises(LookupError, delegation.get_best_proposal)
        self.assertFalse(delegation.has_proposals())

    def test_forbidden_bidders(self):
        delegation = Delegation(goal_wrapper=self.wrapper, auction_id=1)

        bad_name = "forbidden"
        good_name = "good"
        delegation.forbid_bidder(bad_name)
        bad_prop = Proposal(bad_name, 3)
        good_prop = Proposal(good_name, 3)

        self.assertRaises(Warning, delegation.add_proposal, bad_prop)
        self.assertTrue(delegation.is_forbidden(bad_name))
        self.assertFalse(delegation.is_forbidden(good_name))
        delegation.add_proposal(good_prop)  # should not raise anything

        delegation.allow_bidder(bad_name)
        delegation.allow_bidder(good_name)
        self.assertFalse(delegation.is_forbidden(bad_name))
        delegation.add_proposal(bad_prop)   # should not raise anything

        delegation.forbid_bidder(bad_name)
        delegation.reset_forbidden()
        self.assertFalse(delegation.is_forbidden(bad_name))

    def test_steps(self):
        steps = 5
        delegation = Delegation(goal_wrapper=self.wrapper, auction_id=1, auction_steps=steps)

        for i in range(steps - 1):
            self.assertFalse(delegation.decrement_and_check_steps())

        self.assertTrue(delegation.decrement_and_check_steps())

        delegation.reset_steps()
        for i in range(steps - 1):
            self.assertFalse(delegation.decrement_and_check_steps())

        self.assertTrue(delegation.decrement_and_check_steps())

        delegation.reset_steps()
        delegation.end_auction_next_step()
        self.assertTrue(delegation.decrement_and_check_steps())

    def test_goal_interaction(self):
        delegation = Delegation(goal_wrapper=self.wrapper, auction_id=1)

        manager = "manager"
        self.assertEqual(delegation.get_goal_name(), self.goal_name)
        self.assertEqual(delegation.get_goal_representation(), self.wrapper.get_goal_representation())
        delegation.send_goal(name=manager)
        self.assertTrue(self.wrapper.goal_is_created())
        delegation.terminate_goal()
        self.assertFalse(self.wrapper.goal_is_created())

    def test_auction_progress(self):
        auction_id = 1
        delegation = Delegation(goal_wrapper=self.wrapper, auction_id=auction_id)
        self.assertEqual(delegation.get_auction_id(), auction_id)

        bidder = "bidder"
        manager = "manager"

        self.assertTrue(delegation.state.is_ready())
        delegation.start_auction()
        self.assertTrue(delegation.state.is_waiting_for_proposals())

        delegation.make_contract(bidder_name=bidder, manager_name=manager)
        self.assertEqual(delegation.get_contractor(), bidder)
        self.assertTrue(self.wrapper.goal_is_created())

        delegation.finish_delegation()
        self.assertTrue(delegation.state.is_finished())
        self.assertFalse(self.wrapper.goal_is_created())
        self.assertRaises(DelegationContractorError, delegation.get_contractor)

        delegation.make_contract(bidder_name=bidder, manager_name=manager)
        self.assertEqual(delegation.get_contractor(), bidder)
        self.assertTrue(self.wrapper.goal_is_created())

        delegation.fail_current_delegation()
        self.assertTrue(delegation.state.is_ready())
        self.assertRaises(DelegationContractorError, delegation.get_contractor)
        self.assertFalse(self.wrapper.goal_is_created())


class ProposalTest(unittest.TestCase):

    def test_proposal(self):
        name = "test"
        value1 = 1
        proposal1 = Proposal(name=name, value=value1)
        value2 = 3
        proposal2 = Proposal(name=name, value=value2)

        self.assertEqual(proposal1.get_name(), name)
        self.assertEqual(proposal1.get_value(), value1)
        self.assertEqual(cmp(proposal1, proposal2), cmp(value1, value2))


if __name__ == '__main__':
    unittest.main()

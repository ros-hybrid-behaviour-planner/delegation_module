"""
Unit tests for the Generic CostEvaluator

@author: Mengers
"""

from delegation_components.cost_evaluators import CostEvaluatorBase, CostParameters
import unittest
import math


class CostEvaluator(CostEvaluatorBase):
    """
    Version of the CostEvaluator without abstract methods
    """

    def _plan_and_extract_parameters(self, goal_representation, own_name, members, depth, max_depth, task_count, max_task_count):
        test_parameters = CostParameters(base_steps=2, full_steps=4, simple_steps=2, depth=2, max_depth=8,
                                         new_contractor=0, contractor_count=2, new_delegations=1,
                                         num_delegations=1, task_count=1, max_task_count=4)
        return test_parameters


class TestCostEvaluator(unittest.TestCase):
    """
    Unit test for the CostEvaluator
    """

    def setUp(self):
        self.uut = CostEvaluator()
        # setting for these parameters
        self.uut.TASK_UTILIZATION_FACTOR = 1
        self.uut.WORKLOAD_PROPORTION_FACTOR = -0.5
        self.uut.ADDITIONAL_WORKLOAD_FACTOR = 1
        self.uut.ADDITIONAL_DELEGATION_FACTOR = 1
        self.uut.COOPERATION_AMOUNT_FACTOR = 1
        self.uut.CONTRACTOR_NUMBER_FACTOR = 0.5

    def test_cost_eval_base_functions(self):
        """
        Tests base functionality
        """

        self.assertTrue(math.isnan(self.uut.last_cost))
        self.assertEqual(self.uut.last_possibility, False)

    def test_cost_evaluate(self):
        """
        Tests the computing of cost
        """

        test_parameters = CostParameters(base_steps=2, full_steps=4, simple_steps=2, depth=2, max_depth=8,
                                         new_contractor=0, contractor_count=2, new_delegations=1,
                                         num_delegations=1, task_count=1, max_task_count=4)

        cost = self.uut.cost_evaluate(parameters=test_parameters)

        self.assertLess(0, cost)
        self.assertEqual(cost, 10.546875)  # result for these exact parameters

    def test_compute_cost_and_possibility(self):
        """
        Tests full compute cost and possibility function
        """

        # all of these do not matter for the result
        goal_representation = "doesnt matter for this test"
        current_task_count = 1
        max_task_count = 4
        current_depth = 2
        max_depth = 8
        members = ["Member1", "Member2"]
        own_name = "Member1"

        cost, possibility = self.uut.compute_cost_and_possibility(goal_representation=goal_representation, current_task_count=current_task_count,
                                                             max_task_count=max_task_count, current_depth=current_depth,
                                                             max_depth=max_depth, members=members, own_name=own_name)

        self.assertTrue(possibility)
        self.assertLess(0, cost)
        self.assertEqual(cost, 10.546875)  # result for these parameters
        self.assertEqual(self.uut.last_cost, cost)
        self.assertEqual(self.uut.last_possibility, possibility)

    def test_cost_parameter(self):

        # should not raise an exception
        uut = CostParameters(base_steps=2, full_steps=4, simple_steps=2, depth=2, max_depth=8,
                             new_contractor=0, contractor_count=2, new_delegations=1,
                             num_delegations=1, task_count=1, max_task_count=4)

        right_string = "[CostParameters:(name:),(base_steps:2),(full_steps:4),(simple_steps:2)," +\
                       "(depth:2),(max_depth:8),(new_contractor:0),(contractor_count:2),(new_delegation:1)," +\
                       "(num_delegations:1),(task_count:1),(max_task_count:4)]"

        self.assertEqual(uut.__repr__(), right_string)

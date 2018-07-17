#! /usr/bin/env python2

from delegation_tests.cost_evaluator_unit_test import TestCostEvaluator
from delegation_tests.delegation_clients_unit_test import DelegationClientTest
from delegation_tests.delegation_manager_unit_test import DelegationManagerTest
from delegation_tests.delegation_unit_test import DelegationTest, ProposalTest
from delegation_tests.goal_wrappers_unit_test import TestGoalWrapper
from delegation_tests.task_unit_test import TaskTest
import unittest


if __name__ == "__main__":
    # Needs a running ROSCORE
    unittest.main()

#! /usr/bin/env python2
"""
This is just a wrapper that imports all Unittests and runs them as main

Needs a running ROSCORE

@author: Mengers
"""

# "Unused" Imports intended
# noinspection PyUnresolvedReferences
from delegation_tests.cost_evaluator_unit_test import TestCostEvaluator
# noinspection PyUnresolvedReferences
from delegation_tests.delegation_clients_unit_test import DelegationClientTest
# noinspection PyUnresolvedReferences
from delegation_tests.delegation_manager_unit_test import DelegationManagerTest
# noinspection PyUnresolvedReferences
from delegation_tests.delegation_unit_test import DelegationTest, ProposalTest
# noinspection PyUnresolvedReferences
from delegation_tests.goal_wrappers_unit_test import TestGoalWrapper
# noinspection PyUnresolvedReferences
from delegation_tests.task_unit_test import TaskTest
import unittest


if __name__ == "__main__":
    # Needs a running ROSCORE
    unittest.main()

#! /usr/bin/env python2

import unittest
import rospy
import rostest

from delegation_components.delegation_manager import DelegationManager
from decomposition_components.managers import Manager
from delegation_tests.basic_sceanrio import BasicCookingRobot

PKG = "task_decomposition_module"

# TODO THIS NEEDS CHANGING (whole thing)


class DelegationManagerTest1(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(DelegationManagerTest1, self).__init__(*args, **kwargs)
        rospy.init_node("TestNode1")

    def test_basic_auction_with_multiple_bidders(self):

        rospy.loginfo("Starting Basic Test of auctions")

        delegation_manager = DelegationManager(instance_name="Test1", max_tasks=1)

        prefix = "test_manager"

        manager = Manager(prefix=prefix)

        scenarioRobot = BasicCookingRobot(planner_prefix=prefix)

        interface = manager.delegation_client

        interface.register(delegation_manager=delegation_manager)

        for i in range(3):
            manager.step()
            rospy.sleep(1)

        test_condition = scenarioRobot.get_test_cond()

        interface.delegate(goal_name="test_goal", conditions=[test_condition], satisfaction_threshold=1.0)

        for i in range(7):
            manager.step()
            rospy.sleep(1)

        self.assertEqual(delegation_manager.get_delegation(auction_id=1).get_contractor(), "PassiveDelegationManager")


if __name__ == '__main__':
    rostest.rosrun(PKG, 'DelegationManagerTest', DelegationManagerTest1)



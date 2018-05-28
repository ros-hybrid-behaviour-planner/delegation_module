#! /usr/bin/env python2

import unittest
import rospy
import rostest

from delegation_components.delegation_manager import DelegationManager
from delegation_components.delegation import Delegation
from delegation_components.task import Task

PKG = "task_decomposition_module"


class DelegationManagerTest1(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(DelegationManagerTest1, self).__init__(*args, **kwargs)
        rospy.init_node("TestNode1")

    def test_basic_auction_with_multiple_bidders(self):

        rospy.loginfo("Starting Basic Test of auctions")

        self.delegation_manager = DelegationManager("Test1")

        rospy.sleep(5)  # TODO dont understand

        del_id = self.delegation_manager.delegate("test_goal_auction")

        test_delegation = self.delegation_manager.get_delegation(del_id)

        rospy.sleep(5)

        self.delegation_manager.end_auction(test_delegation)

        rospy.sleep(2)

        self.assertNotEqual(test_delegation.get_contractor(), "")

        # Test terminate

        rospy.loginfo("Starting Test Terminate")

        while not self.delegation_manager._got_task:
            # make new auctions until u have a task yourself
            del_id = self.delegation_manager.delegate("test_goal_auction")

            test_delegation = self.delegation_manager.get_delegation(del_id)

            rospy.sleep(3)

            self.delegation_manager.end_auction(test_delegation)

            rospy.sleep(2)

        test_id = self.delegation_manager.get_task().get_auction_id()

        test_delegation = self.delegation_manager.get_delegation(test_id)

        self.delegation_manager.terminate(test_delegation)

        rospy.sleep(2)

        self.assertFalse(self.delegation_manager._got_task)

        # Test failure

        rospy.loginfo("Starting Test Failure")

        while not self.delegation_manager._got_task:
            # make new auctions until u have a task yourself
            del_id = self.delegation_manager.delegate("test_goal_auction")

            test_delegation = self.delegation_manager.get_delegation(del_id)

            rospy.sleep(2)

            self.delegation_manager.end_auction(test_delegation)

            rospy.sleep(2)

        test_id = self.delegation_manager.get_task().get_auction_id()

        test_delegation = self.delegation_manager.get_delegation(test_id)

        self.delegation_manager.failure()

        rospy.sleep(2)

        self.assertTrue(test_delegation.state.is_waiting_for_proposals())



if __name__ == '__main__':
    rostest.rosrun(PKG, 'DelegationManagerTest', DelegationManagerTest1)



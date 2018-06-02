#! /usr/bin/env python2

import unittest
import rospy
import rostest

from behaviour_components.managers import Manager
from delegation_components.cost_computing import PDDLCostEvaluator
from rostests.basic_sceanrio import BasicCookingRobot

PKG = "task_decomposition_module"


class CostComputingTest(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(CostComputingTest, self).__init__(*args, **kwargs)
        rospy.init_node("TestNode1")

    def test_basic(self):
        prefix = ""

        m = Manager(prefix=prefix)

        bcr = BasicCookingRobot(planner_prefix=prefix)

        m.add_goal(bcr.get_test_goal())

        for i in range(7):
            m.step()
            rospy.sleep(1)

        goal = bcr.get_goal()

        cost_computer = PDDLCostEvaluator(planning_function=m.plan_with_additional_goal)

        cost, possible = cost_computer.compute_cost_and_possibility(goal.fetchPDDL()[0].statement)

        print ("Possibility:"+str(possible)+" , Cost:"+str(cost))

        self.assertTrue(possible)
        self.assertLess(0, cost)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'CostComputingTest', CostComputingTest)

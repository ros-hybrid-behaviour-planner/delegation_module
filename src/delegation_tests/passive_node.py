#! /usr/bin/env python2

import rospy

from delegation_components.delegation_manager import DelegationManager
from behaviour_components.managers import Manager
from delegation_tests.basic_sceanrio import BasicCookingRobot
from delegation_components.cost_computing import AbstractCostEvaluator


class AlwaysWinningCostEvaluator(AbstractCostEvaluator):

    def __init__(self):
        super(AlwaysWinningCostEvaluator, self).__init__()

    def compute_cost_and_possibility(self, goal_representation):
        self._last_cost, self._last_possibility = 0, True
        return 0, True


if __name__ == '__main__':

    rospy.init_node(name="PassiveNode")

    rospy.loginfo("Starting a passive node with the basic scenario")

    delegation_manager = DelegationManager(instance_name="PassiveDelegationManager", max_tasks=10)

    prefix = "PassiveManager"

    manager = Manager(prefix=prefix)

    scenarioRobot = BasicCookingRobot(planner_prefix=prefix)

    interface = manager.get_delegation_interface()

    interface.register(delegation_manager=delegation_manager, add_own_cost_evaluator=False)

    evaluator = AlwaysWinningCostEvaluator()

    delegation_manager.set_cost_function_evaluator(cost_function_evaluator=evaluator, manager_name=prefix)

    while not rospy.is_shutdown():
        try:
            manager.step()
        except Exception as e:
            rospy.logwarn("Exception in PassiveNode step: " + e.message)
        rospy.sleep(1)




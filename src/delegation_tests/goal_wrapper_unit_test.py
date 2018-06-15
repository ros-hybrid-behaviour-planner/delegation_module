import unittest
import rospy

from behaviour_components.managers import Manager
from behaviour_components.sensors import TopicSensor
from behaviour_components.conditions import Condition
from behaviour_components.activators import BooleanActivator
from behaviour_components.goals import OfflineGoal
from std_msgs.msg import Bool

from delegation_components.goalwrapper import RHBPGoalWrapper


class GoalWrapperTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        super(GoalWrapperTest, cls).setUpClass()
        rospy.init_node("test_node")
        cls.manager = Manager(prefix="Test_manager")
        sensor = TopicSensor(name="test_sensor", topic="sensor_topic", message_type=Bool, initial_value=False)
        cls.test_condition = Condition(sensor, BooleanActivator())
        cls.comparison_goal = OfflineGoal(name="comparison_goal", planner_prefix="Test_manager")

    def test_basic_setter_getter(self):
        test_wrapper = RHBPGoalWrapper(name="test_goal", conditions=[self.test_condition])

        self.assertEqual(test_wrapper.get_goal_name(), "test_goal")
        self.assertFalse(test_wrapper.goal_is_created())

        exception = False
        try:
            test_wrapper.get_goal()
        except RuntimeError:
            exception = True

        self.assertTrue(exception)

    def test_goal_representation(self):
        test_wrapper = RHBPGoalWrapper(name="test_goal", conditions=[self.test_condition])

        string = self.test_condition.getPreconditionPDDL(1.0).statement
        self.assertEqual(test_wrapper.get_goal_representation(), string)

    def test_sending_goal(self):
        test_wrapper = RHBPGoalWrapper(name="test_goal", conditions=[self.test_condition])

        print(test_wrapper.goal_is_created())

        test_wrapper.send_goal(name="Test_manager")

        rospy.sleep(1)

        print(test_wrapper.goal_is_created())

        self.assertTrue(test_wrapper.goal_is_created())

        goal = test_wrapper.get_goal()  # check if exception is raised

        goals = self.manager.goals

        manager_has_goal = False
        for g in goals:
            if g._name == "test_goal":
                manager_has_goal = True

        self.assertTrue(manager_has_goal)

    def test_terminating_goal(self):
        test_wrapper = RHBPGoalWrapper(name="terminate_goal", conditions=[self.test_condition])

        test_wrapper.send_goal(name="Test_manager")

        rospy.sleep(1)

        test_wrapper.terminate_goal()

        rospy.sleep(1)

        goals = self.manager.goals

        manager_has_goal = False
        for g in goals:
            if g._name == "terminate_goal":
                manager_has_goal = True

        self.assertFalse(manager_has_goal)


if __name__ == '__main__':
    unittest.main()
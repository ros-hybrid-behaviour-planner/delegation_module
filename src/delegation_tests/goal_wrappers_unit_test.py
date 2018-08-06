import unittest

from delegation_components.goal_wrappers import GoalWrapperBase


class GoalWrapper(GoalWrapperBase):

    def get_goal_representation(self):
        return

    def send_goal(self, name):
        self._created_goal = True
        self._goal = name
        return

    def terminate_goal(self):
        return

    def check_if_still_alive(self):
        return True

    def check_goal_finished(self):
        return True


class TestGoalWrapper(unittest.TestCase):

    def test_base_functions(self):
        test_name = "Test"
        test_goal = "test_goal"
        uut = GoalWrapper(name=test_name)

        self.assertEqual(test_name, uut.goal_name)
        self.assertFalse(uut.goal_is_created())
        self.assertRaises(RuntimeError, uut.get_goal)

        uut.send_goal(name=test_goal)

        self.assertTrue(uut.goal_is_created())
        self.assertEqual(uut.get_goal(), test_goal)

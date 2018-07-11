
from delegation_components.cost_evaluators import CostEvaluatorBase
import unittest


class CostEvaluator(CostEvaluatorBase):

    def compute_cost_and_possibility(self, goal_representation):
        self._last_possibility = True
        self._last_cost = 5


class TestCostEvaluator(unittest.TestCase):

    def test_cost_eval_base_functions(self):
        uut = CostEvaluator()

        self.assertEqual(uut.last_cost, -1)
        self.assertEqual(uut.last_possibility, False)

        uut.compute_cost_and_possibility(goal_representation="this doesnt matter")

        self.assertEqual(uut.last_cost, 5)
        self.assertEqual(uut.last_possibility, True)

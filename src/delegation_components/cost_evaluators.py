
from abc import ABCMeta, abstractmethod


class CostEvaluatorBase(object):
    """
    Base Class for evaluators of cost functions for a DelegationManager
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        """
        Constructor, gives possibility and cost initial values
        """

        self._last_cost = -1
        self._last_possibility = False

    @abstractmethod
    def compute_cost_and_possibility(self, goal_representation, current_task_count, max_task_count, current_depth, max_depth):
        """
        Computes the cost and possibility of fulfilling a goal
        on this platform

        :param goal_representation: representation of the goal
                that should be fulfilled
        :return: Cost, Possibility (Float, Boolean)
        """

        raise NotImplementedError

    @property
    def last_cost(self):
        """
        Returns the last computed cost

        :return: the last computed cost
        """

        return self._last_cost

    @property
    def last_possibility(self):
        """
        Returns the last computed possibility

        :return: whether in last computation the
                task was possible
        """

        return self._last_possibility

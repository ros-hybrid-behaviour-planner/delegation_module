"""
Abstract GoalWrapper

@author: Mengers
"""

from abc import ABCMeta, abstractmethod


class GoalWrapperBase(object):
    """
    Abstract Base class for all GoalWrappers
    """

    __metaclass__ = ABCMeta

    def __init__(self, name):
        """
        Constructor that initiates all members that are
        needed by all GoalWrappers

        :param name: name of the goal
        :type name: str
        """

        self._name = name
        self._created_goal = False
        self._goal = None

    def __del__(self):
        """
        Destructor
        """

        if self._created_goal:
            del self._goal

    @property
    def goal_name(self):
        """
        Returns the name of the corresponding goal

        :return: name of the goal
        :rtype: str
        """

        return self._name

    def goal_is_created(self):
        """
        Checks if a goal has been created

        :return: whether a goal has been created or not
        :rtype: bool
        """

        return self._created_goal

    def get_goal(self):
        """
        Gets the goal if created or raises an Exception

        :return: instance of the goal of this wrapper
        :rtype: Goal
        :raises RuntimeError: if no goal has been created yet
        """

        if not self._created_goal:
            raise RuntimeError("Trying to access a goal, while the goal has not been created")
        else:
            return self._goal

    # ------ Abstract Methods ------

    @abstractmethod
    def get_goal_representation(self):
        """
        Returns a Representation of a goal, that can be used
        to evaluate the cost of accomplishing this goal by some
        kind of CostEvaluatorBase

        Needs to be overridden!

        :return: the goal representation
        :rtype: str
        """

        raise NotImplementedError

    @abstractmethod
    def send_goal(self, name):
        """
        Takes care of sending the goal towards the right agent.
        The way this can be accomplished depends on the structure
        of the system

        Needs to be overridden!

        :param name: name of the manager that the goal is sent to
        :type name: str
        """

        raise NotImplementedError

    @abstractmethod
    def terminate_goal(self):
        """
        Terminates the created goal
        """

        raise NotImplementedError

    @abstractmethod
    def check_if_still_alive(self):
        """
        Checks whether this goal is still actively pursued by the contractor

        :return: if it is actively pursued
        :rtype: bool
        """

        raise NotImplementedError

    @abstractmethod
    def check_goal_finished(self):
        """
        Checks whether the goal is finished or not

        :return: if the goal is finished or not
        :rtype: bool
        """

        raise NotImplementedError

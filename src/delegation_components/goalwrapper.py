
from behaviour_components.goals import GoalBase
from delegation_components.delegation_errors import DelegationError
from abc import ABCMeta, abstractmethod


class GoalWrapperBase(object):
    """
    Base class for all GoalWrapper used in this package
    """

    __metaclass__ = ABCMeta

    def __init__(self, name):
        """
        Constructor that initiates all member that are
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

    def get_goal_name(self):
        """
        Returns the name of the corresponding goal

        :return: name of the goal
        :rtype: str
        """

        return self._name

    def get_goal(self):
        """
        Gets the goal if created or raises an Exception

        :return: instance of the goal of this wrapper
        :raises RuntimeError: if no goal has been created yet
        """

        if not self._created_goal:
            raise RuntimeError("Trying to access a goal, while the goal has not been created")
        else:
            return self._goal

    def goal_is_created(self):
        """
        Checks if a goal has been created

        :return: whether a goal has been created or not
        :rtype: bool
        """

        return self._created_goal

    @abstractmethod
    def get_goal_representation(self):
        """
        Returns a Representation of a goal, that can be used
        to evaluate the cost of accomplishing this goal by some
        kind of AbstractCostEvaluator

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


class RHBPGoalWrapper(GoalWrapperBase):
    """
    Class that wraps goals that are used in the RHBP.
    Uses PDDL-based goal representations and sends goals
    by registering them at the corresponding manager.
    """

    def __init__(self, name, conditions, satisfaction_threshold=1.0):
        """
        Constructor

        :param name: name of the goal
        :param conditions: list of conditions of this goals
        :param satisfaction_threshold: satisfaction threshold
                of the goal
        """

        super(RHBPGoalWrapper, self).__init__(name=name)

        self._conditions = conditions
        self._satisfaction_threshold = satisfaction_threshold

    def __del__(self):
        """
        Destructor
        """

        super(RHBPGoalWrapper, self).__del__()
        del self._conditions

    def get_goal_representation(self):
        """
        Returns the representation of the goal in form
        of a PDDL goal-statement

        :return: goal representing PDDL-String
        """

        return " ".join([x.getPreconditionPDDL(self._satisfaction_threshold).statement for x in self._conditions])

    def send_goal(self, name=""):
        """
        Sends the goal to the RHBP-manager with this planner-prefix
        by registering it directly at the manager

        :param name: prefix of the manager, that should
                receive the goal
        :raises DelegationError: if there is any problem with the goal creation
        """
        try:
            self._goal = GoalBase(name=self.get_goal_name(), plannerPrefix=name, conditions=self._conditions, satisfaction_threshold=self._satisfaction_threshold)
            self._created_goal = True
            return
        except Exception as e:
            self._created_goal = False
            self._goal = None
            raise DelegationError("Sending goal raised exception: " + e.message)

    def terminate_goal(self):
        """
        Terminates (unregister and deletes) the created goal
        """

        if self.goal_is_created():
            self._created_goal = False
            self._goal.__del__()   # unregisters goal
            self._goal = None

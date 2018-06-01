
from behaviour_components.goals import GoalBase


class GoalWrapperBase(object):
    """
    Base class for all GoalWrapper used in this package
    """

    def __init__(self, name):
        """
        Constructor that initiates all member that are
        needed by all GoalWrappers

        :param name: name of the goal
        """
        self.__name = name
        self.__created_goal = False
        self.__goal = None

    def get_goal_representation(self):
        """
        Returns a Representation of a goal, that can be used
        to evaluate the cost of accomplishing this goal by some
        kind of AbstractCostEvaluator

        Needs to be overridden!

        :return: the goal representation
        """

        raise NotImplementedError

    def send_goal(self, name):
        """
        Takes care of sending the goal towards the right agent.
        The way this can be accomplished depends on the structure
        of the system

        Needs to be overridden!
        """

        raise NotImplementedError

    def get_goal(self):
        """
        Gets the goal if created or raises an Exception

        :return: instance of the goal of this wrapper
        :raises RuntimeError: if no goal has been created yet
        """

        if not self.__created_goal:
            raise RuntimeError("Trying to access a goal, while the goal has not been created")
        else:
            return self.__goal

    def goal_is_created(self):
        """
        Checks if a goal has been created

        :return: whether a goal has been created or not
        """

        return self.__created_goal


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

        self.__conditions = conditions
        self.__satisfaction_threshold = satisfaction_threshold

    def get_goal_representation(self):
        """
        Returns the representation of the goal in form
        of a PDDL goal-statement

        :return: goal representing PDDL-String
        """

        return " ".join([x.getPreconditionPDDL(self.__satisfaction_threshold).statement for x in self.__conditions])

    def send_goal(self, name=""):
        """
        Sends the goal to the RHBP-manager with this planner-prefix
        by registering it directly at the manager

        :param name: prefix of the manager, that should
                receive the goal
        """

        self.__goal = GoalBase(name=self.__name, plannerPrefix=name, conditions=self.__conditions, satisfaction_threshold=self.__satisfaction_threshold)
        self.__created_goal = True
        # TODO raise exception if it doesnt work


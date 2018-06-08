
from delegation_errors import DelegationPlanningWarning


class AbstractCostEvaluator(object):
    """
    Base Class for evaluators of cost functions for a DelegationManager
    """

    def __init__(self):
        """
        Constructor, gives possibility and cost initial values
        """

        self.__last_cost = -1
        self.__last_possibility = False

    def get_cost(self):
        """
        Returns the last computed cost

        :return: the last computed cost
        """

        return self.__last_cost

    def get_possibility(self):
        """
        Returns the last computed possibility

        :return: whether in last computation the
                task was possible
        """

        return self.__last_possibility

    def compute_cost_and_possibility(self, goal_representation):
        """
        Computes the cost and possibility of fulfilling a goal
        on this platform

        :param goal_representation: representation of the goal
                that should be fulfilled
        :return: Cost, Possibility (Float, Boolean)
        """

        raise NotImplementedError


class PDDLCostEvaluator(AbstractCostEvaluator):
    """
    Cost function evaluator for DelegationManager using the symbolic
    PDDL-Planer of the RHBP-Manager
    """

    def __init__(self, planning_function):
        """
        Constructor

        :param planning_function: functionpointer on a function, that
                returns a PDDL plan and takes a goal-statement as input
        """

        super(PDDLCostEvaluator, self).__init__()
        self.__planning_function = planning_function

    def compute_cost_and_possibility(self, goal_representation):
        """
        Computes cost and possibility of a goal given its statement.
        This is accomplished by trying to plan with the PDDL-planing-function
        and using this plan to extract cost/possibility

        :param goal_representation: PDDL representation of the goal (goal-statement)
        :return: Cost, Possibility for this goal
        :raises DelegationPlanningWarning: if planning failed
        """

        possible = False
        cost = -1
        try:
            plan = self.__planning_function(goal_representation)
            if plan and "cost" in plan and plan["cost"] != -1.0:
                possible = True
                cost = self.__compute_cost(plan=plan)
        except Exception as e:  # catch any exception
            raise DelegationPlanningWarning(str(e.message))

        self.__last_cost, self.__last_possibility = cost, possible

        return cost, possible

    def __compute_cost(self, plan):
        """
        Extract all needed information out of the plan and
        computes cost

        :param plan: proper PDDL plan
        :return: the cost that was computed
        """

        steps = plan["cost"]

        # TODO create smart heuristic for cost based on steps and possible other stuff

        return steps

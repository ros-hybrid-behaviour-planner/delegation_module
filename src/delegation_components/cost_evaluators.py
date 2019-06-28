"""
Abstract CostEvaluator with the default cost_function

@author: Mengers
"""

from abc import ABCMeta, abstractmethod
from delegation_errors import DelegationPlanningWarning
import rospy
import sys


class CostEvaluatorBase(object):
    """
    Base Class for evaluators of cost functions for a DelegationManager
    """

    __metaclass__ = ABCMeta

    IMPOSSIBLE_COSTS = float('nan')

    TASK_UTILIZATION_FACTOR = 1
    WORKLOAD_PROPORTION_FACTOR = -0.025
    ADDITIONAL_WORKLOAD_FACTOR = 1
    ADDITIONAL_DELEGATION_FACTOR = 1
    COOPERATION_AMOUNT_FACTOR = 1
    CONTRACTOR_NUMBER_FACTOR = 0.1

    @classmethod
    def update_config(cls, **kwargs):
        """
        Updates cost factors for all CostEvaluators on this node

        :param kwargs: dict of weight-factors
        :type kwargs: dict
        :return: string that can be used for logging, includes all information
            about the new factors
        :rtype: str
        """

        cls.TASK_UTILIZATION_FACTOR = kwargs.get("task_utilization_factor", cls.TASK_UTILIZATION_FACTOR)
        cls.WORKLOAD_PROPORTION_FACTOR = kwargs.get("workload_proportion_factor", cls.WORKLOAD_PROPORTION_FACTOR)
        cls.ADDITIONAL_WORKLOAD_FACTOR = kwargs.get("additional_workload_factor", cls.ADDITIONAL_WORKLOAD_FACTOR)
        cls.ADDITIONAL_DELEGATION_FACTOR = kwargs.get("additional_delegation_factor", cls.ADDITIONAL_DELEGATION_FACTOR)
        cls.COOPERATION_AMOUNT_FACTOR = kwargs.get("cooperation_amount_factor", cls.COOPERATION_AMOUNT_FACTOR)
        cls.CONTRACTOR_NUMBER_FACTOR = kwargs.get("contractor_number_factor", cls.CONTRACTOR_NUMBER_FACTOR)

        log_string = "CostFunction parameters updated:" +\
                    "\n\tTASK_UTILIZATION_FACTOR\t" + str(cls.TASK_UTILIZATION_FACTOR) +\
                    "\n\tWORKLOAD_PROPORTION_FACTOR\t" + str(cls.WORKLOAD_PROPORTION_FACTOR) +\
                    "\n\tADDITIONAL_WORKLOAD_FACTOR\t" + str(cls.ADDITIONAL_WORKLOAD_FACTOR) +\
                    "\n\tADDITIONAL_DELEGATION_FACTOR\t" + str(cls.ADDITIONAL_DELEGATION_FACTOR) +\
                    "\n\tCOOPERATION_AMOUNT_FACTOR\t" + str(cls.COOPERATION_AMOUNT_FACTOR) +\
                    "\n\tCONTRACTOR_NUMBER_FACTOR\t" + str(cls.CONTRACTOR_NUMBER_FACTOR)

        return log_string

    def __init__(self):
        """
        Constructor, gives possibility and cost initial values
        """

        self._last_cost = CostEvaluatorBase.IMPOSSIBLE_COSTS
        self._last_possibility = False

    def compute_cost_and_possibility(self, goal_representation, current_task_count, max_task_count, current_depth, max_depth, members, own_name):
        """
        Computes cost and possibility of a goal given its statement.
        This is accomplished by trying to plan with the
        plan_and_extract_parameters() function and uses the parameters for the
        cost_evaluate() function

        :param goal_representation: PDDL representation of the goal (goal-statement)
        :type goal_representation: str
        :param current_task_count: number of tasks currently running
        :type current_task_count: int
        :param max_task_count: maximum number of tasks
        :type max_task_count: int
        :param current_depth: current depth of this task
        :type current_depth: int
        :param max_depth: maximum depth for tasks
        :type max_depth: int
        :param members: list of the names of current members of this delegation
        :type members: list(str)
        :param own_name: name of this unit (like the names in members)
        :type own_name: str
        :return: Cost, Possibility for this goal
        :rtype: float, bool
        :raises DelegationPlanningWarning: if planning failed
        """

        self._last_possibility = False
        self._last_cost = CostEvaluatorBase.IMPOSSIBLE_COSTS
        try:
            parameters = self._plan_and_extract_parameters(goal_representation=goal_representation, own_name=own_name,
                                                           members=members, depth=current_depth, max_depth=max_depth,
                                                           task_count=current_task_count, max_task_count=max_task_count)

            self._last_possibility = True
            self._last_cost = self.cost_evaluate(parameters=parameters)

        except Exception as e:  # catch any exception
            self._last_possibility = False
            raise DelegationPlanningWarning(str(e.message))

        return self._last_cost, self._last_possibility

    def cost_evaluate(self, parameters):
        """
        Evaluates cost with the default cost_function

        Can be overridden if other cost_function is needed

        :param parameters: parameters for the cost function
        :type parameters: CostParameters
        :return: computed cost
        :rtype: float
        """

        # to make sure infinite maximums do not do any harm
        max_tasks = sys.maxint if parameters.max_task_count < 0 else parameters.max_task_count
        max_depth = sys.maxint if parameters.max_depth < 0 else parameters.max_depth

        task_capacity_utilization = (1 + self.TASK_UTILIZATION_FACTOR *
                                     float(parameters.task_count) / float(max_tasks))
        workload_proportion = (1 + self.WORKLOAD_PROPORTION_FACTOR *
                               float(parameters.simple_steps) / float(parameters.full_steps))
        additional_workload = (1 + self.ADDITIONAL_WORKLOAD_FACTOR *
                               float(parameters.base_steps) / float(parameters.full_steps))
        additional_depth = (1 + self.ADDITIONAL_DELEGATION_FACTOR *
                            float(parameters.new_delegations) * float(parameters.depth) / float(max_depth))
        cooperation_amount = (1 + self.COOPERATION_AMOUNT_FACTOR *
                              float(parameters.num_delegations) / float(parameters.simple_steps))
        contractor_number = (1 + self.CONTRACTOR_NUMBER_FACTOR *
                             float(parameters.new_contractor) * (1 - float(parameters.contractor_count) / (float(parameters.depth) + 1)))

        cost = parameters.full_steps * task_capacity_utilization * workload_proportion * additional_workload * additional_depth * cooperation_amount * contractor_number

        self._log_cost_computed(name=parameters.name, cost=cost, full_steps=parameters.full_steps,
                                task_capacity_utilization=task_capacity_utilization,
                                workload_proportion=workload_proportion, additional_workload=additional_workload,
                                additional_depth=additional_depth, cooperation_amount=cooperation_amount,
                                contractor_number=contractor_number)

        return cost

    @abstractmethod
    def _plan_and_extract_parameters(self, goal_representation, own_name, members, depth, max_depth, task_count, max_task_count):
        """
        Here the planning should happen depending on system planning
        capabilities, afterwards needed parameters for cost function should
        be determined and packed into a CostParamters object

        Needs to be implemented!

        :param goal_representation: representation of the goal
        :type goal_representation: str
        :param own_name: name of the DelegationManager
        :type own_name: str
        :param members: current members of this delegation
        :type members: list
        :param depth: current depth of this delegation
        :type depth: int
        :param max_depth: max depth of this delegation
        :type max_depth: int
        :param task_count: current task count of this DelegationManager
        :type task_count: int
        :param max_task_count: max task count of this DelegationManager
        :type max_task_count: int
        :return: CostFunction-Parameters
        :rtype: CostParameters
        :raises: Exception if planning impossible
        """

        raise NotImplementedError

    def _log_cost_computed(self, name, cost, full_steps, task_capacity_utilization, workload_proportion, additional_workload, additional_depth, cooperation_amount, contractor_number):
        """
        Logs the information about the cost computed

        :param name: name of this DelegationManager
        :type name: str
        :param cost: cost computed
        :type cost: float
        :param full_steps: steps of the full plan
        :type full_steps: int
        :param task_capacity_utilization: task_capacity_utilization-factor value
        :type task_capacity_utilization: float
        :param workload_proportion:workload_proportion-factor value
        :type workload_proportion: float
        :param additional_workload:additional_workload-factor value
        :type additional_workload: float
        :param additional_depth:additional_depth-factor value
        :type additional_depth: float
        :param cooperation_amount:cooperation_amount-factor value
        :type cooperation_amount: float
        :param contractor_number:contractor_number-factor value
        :type contractor_number: float
        """

        rospy.loginfo(name + ": Computed cost for accomplishing the goal from following factors:"
                           + "\n\tSteps of full plan " + str(full_steps)
                           + "\n\tTask Capacity Utilization " + str(task_capacity_utilization)
                           + "\n\tWorkload Proportion " + str(workload_proportion)
                           + "\n\tAdditional Workload " + str(additional_workload)
                           + "\n\tAdditional Delegation Depth " + str(additional_depth)
                           + "\n\tCooperation Amount " + str(cooperation_amount)
                           + "\n\tNumber of Contractors " + str(contractor_number)
                           + "\nResulting Cost (product of the above): " + str(cost))

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


class CostParameters(object):
    """
    Container object for parameters used by the cost function
    """

    def __init__(self, base_steps, full_steps, simple_steps, depth,
                 max_depth, new_contractor, contractor_count, new_delegations,
                 num_delegations, task_count, max_task_count, name=""):
        """
        Constructor

        :param base_steps: steps of the base plan
        :type base_steps: int
        :param full_steps: steps of the full plan
        :type full_steps: int
        :param simple_steps: steps of the simple plan
        :type simple_steps: int
        :param depth: current depth of the delegation
        :type depth: int
        :param max_depth: max depth of the delegation
        :type max_depth: int
        :param new_contractor: whether there would be a new contractor or not
        :type new_contractor: float
        :param contractor_count: number of current contractors
        :type contractor_count: int
        :param new_delegations: whether there would be new delegations or not
        :type new_delegations: float
        :param num_delegations: number of used delegations
        :type num_delegations: int
        :param task_count: current number of tasks on this DelegationManager
        :type task_count: int
        :param max_task_count: max number of tasks
        :type max_task_count: int
        :param name: name of the corresponding DelegationManager, can be empty
        :type name: str
        """

        self.name = name
        self.base_steps = base_steps
        self.full_steps = full_steps
        self.simple_steps = simple_steps
        self.depth = depth
        self.max_depth = max_depth
        self.new_contractor = new_contractor
        self.contractor_count = contractor_count
        self.new_delegations = new_delegations
        self.num_delegations = num_delegations
        self.task_count = task_count
        self.max_task_count = max_task_count

    def __repr__(self):
        """
        Representation
        :return: Representation
        :rtype: str
        """

        representation = "[CostParameters:(name:" + self.name
        representation += "),(base_steps:" + str(self.base_steps)
        representation += "),(full_steps:" + str(self.full_steps)
        representation += "),(simple_steps:" + str(self.simple_steps)
        representation += "),(depth:" + str(self.depth)
        representation += "),(max_depth:" + str(self.max_depth)
        representation += "),(new_contractor:" + str(self.new_contractor)
        representation += "),(contractor_count:" + str(self.contractor_count)
        representation += "),(new_delegation:" + str(self.new_delegations)
        representation += "),(num_delegations:" + str(self.num_delegations)
        representation += "),(task_count:" + str(self.task_count)
        representation += "),(max_task_count:" + str(self.max_task_count)
        representation += ")]"
        return representation

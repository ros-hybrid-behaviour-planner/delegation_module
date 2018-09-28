"""
Abstract DelegationClient, that is the part of the DelegationModule that
interfaces with outside object

@author: Mengers
"""

from abc import abstractmethod, ABCMeta
import rospy
from delegation_components.delegation_errors import DelegationError


class DelegationClientBase(object):
    """
    Client for the DelegationManager

    The part of the DelegationModule that directly interfaces
    with outside objects.
    """

    __metaclass__ = ABCMeta

    # ------ Class Members ------
    instance_counter = 0
    all_clients_and_ids = {}
    logger = rospy

    # ------ Class Methods ------

    @classmethod
    def unregister_at(cls, client_ids):
        """
        Unregisters the DelegationManager at all clients with IDs in the given
        list

        :param client_ids: list of client IDs
        :type client_ids: list
        """

        for i in client_ids:
            try:
                cls.all_clients_and_ids[i].unregister()
            except Exception as e:
                cls.logger.logwarn(msg="Error in unregistering of a DelegationManager at client with ID "
                                       + str(i) + ": " + str(e.message))
                # not essential for running, so continue

    @classmethod
    def get_client(cls, client_id):
        """
        Returns the client with the given ID

        :param client_id: ID of the client
        :type client_id: int
        :return: matching instance of client
        :rtype: DelegationClientBase
        """

        return cls.all_clients_and_ids[client_id]

    # ------ Constructor/Destructor ------

    def __init__(self):
        """
        Constructor
        """

        self._delegation_manager = None
        self._manager_active = False
        self._active_delegations = []  # list of IDs
        DelegationClientBase.instance_counter += 1
        self._client_id = DelegationClientBase.instance_counter
        DelegationClientBase.all_clients_and_ids[self._client_id] = self

    def __del__(self):
        """
        Destructor
        """

        self.unregister()

    # ------ Registration of DelegationManagers ------

    def register(self, delegation_manager):
        """
        Registers a delegation manager at this client if there was none
        registered till now, else the old one will still be used

        If wanted this can be overridden with a version that additionally
        invokes add_own_cost_evaluator() with a suiting kind of
        CostEvaluatorBase

        :param delegation_manager: DelegationManager
        :type delegation_manager: DelegationManager
        """

        if self._manager_active:
            self.logger.logwarn("Attempt to log a new delegation_manager with the name \""
                                + str(delegation_manager.name)
                                + "\" while one with the name \""
                                + str(self._delegation_manager.name)
                                + "\" is already registered.\nNew DelegationManager will be ignored.")
            # will still use the old registered one
            return

        self._delegation_manager = delegation_manager
        self._manager_active = True
        delegation_manager.add_client(client_id=self._client_id)

    def unregister(self):
        """
        Unregisters currently used DelegationManager from this client
        """

        if self._manager_active:
            self._delegation_manager.remove_client(client_id=self._client_id)
            self._manager_active = False
            self._delegation_manager = None

    def check_if_registered(self):
        """
        Checks whether there is currently a registered DelegationManager or not

        :return: whether there is currently a registered DelegationManager or
                not
        :rtype: bool
        """

        return self._manager_active

    def add_own_cost_evaluator(self, cost_evaluator, agent_name):
        """
        Adds the given cost_evaluator to the delegation_manager if one is
        present

        :param cost_evaluator: A kind of CostEvaluatorBase
        :type cost_evaluator: CostEvaluatorBase
        :param agent_name: Name of the corresponding agent
        :type agent_name: str
        """

        if self._manager_active:
            self._delegation_manager.set_cost_function_evaluator(cost_function_evaluator=cost_evaluator,
                                                                 agent_name=agent_name,
                                                                 client_id=self._client_id)

    # ------ Making sure the Delegations are all up to date ------

    def do_step(self):
        """
        If a delegation manager is registered, it will make a step for all its
        own delegations at this DelegationManager
        """

        if self._manager_active:
            self._delegation_manager.do_step(delegation_ids=self._active_delegations)

    def notify_goal_removal(self, goal_name):
        """
        Notifies the DelegationManager, if present, that a goal was removed.
        So that he is notified if the goal of a task was terminated

        :param goal_name: name of the removed goal
        :type goal_name: str
        """

        if self._manager_active:
            self._delegation_manager.end_task(goal_name=goal_name)

    # ------ Delegations ------

    def delegate_goal_wrapper(self, goal_wrapper, own_cost=-1, known_depth=None):
        """
        Tries to delegate the wrapped goal if a DelegationManager is registered

        :param goal_wrapper: An kind of an GoalWrapperBase or its children
        :type goal_wrapper: GoalWrapperBase
        :param own_cost: cost if i could do this task myself for the given cost.
                This should be negative if i can not do the task myself
        :type own_cost: float
        :param known_depth: if a specific delegation depth is known for this
                delegation, this depth should be given here
        :type known_depth: int
        :return: ID of the delegation
        :rtype: int
        :raises RuntimeError: if no DelegationManager is registered
        :raises DelegationError: if the Delegation-attempt was not successful
        """

        if not self._manager_active:
            raise RuntimeError("Delegation without a registered DelegationManager")

        try:
            delegation_id = self._delegation_manager.delegate(goal_wrapper=goal_wrapper,
                                                              client_id=self._client_id,
                                                              own_cost=own_cost,
                                                              known_depth=known_depth)
        except DelegationError:
            raise   # forward the DelegationError

        self._active_delegations.append(delegation_id)
        return delegation_id

    def terminate_delegation(self, delegation_id):
        """
        Terminates the delegation with the given ID

        :param delegation_id: ID of the delegation
        :type delegation_id: int
        """

        if self._manager_active and self._active_delegations.__contains__(delegation_id):
            self._delegation_manager.terminate(auction_id=delegation_id)
            self._active_delegations.remove(delegation_id)

    def terminate_all_delegations(self):
        """
        Terminates all current delegations of this client
        """

        delegations = self._active_delegations[:]   # copy of the list
        for delegation_id in delegations:
            self.terminate_delegation(delegation_id=delegation_id)

    @abstractmethod
    def delegate(self, goal_name):
        """
        Tries to delegate a goal with given parameters

        Creates a suiting GoalWrapper and invokes delegate_goal_wrapper with
        this goal wrapper, also determines needed parameters like the
        known depth or the own_cost, if applicable

        Needs to be overridden!

        :param goal_name: name of the goal
        :type goal_name: str
        :return: the ID of the delegation that was created
        :rtype: int
        :raises RuntimeError: if no DelegationManager is registered
        """

        if not self._manager_active:
            raise RuntimeError("Delegation without a registered DelegationManager")

        raise NotImplementedError

    @abstractmethod
    def start_work_for_delegation(self, delegation_id):
        """
        This is needed if own delegation attempts can come back (a own_cost was
        given at delegation_start) and will be invoked if the delegation is won
        by the own proposal

        Needs to make sure the work is done myself

        Needs to be overridden!

        :param delegation_id: ID of the delegation that i need to start working
                myself for
        :type delegation_id: int
        :return: None
        """

        raise NotImplementedError

    @abstractmethod
    def delegation_successful(self, delegation_id):
        """
        This method will be invoked if the goal_wrapper of the delegation with
        the given delegation id signals that the goal has been fulfilled
        completely

        Here can be done work, if work is needed after a successful delegation,
        else just pass

        Needs to be overridden!

        :param delegation_id: ID of the delegation that was successful
        :type delegation_id: int
        :return: None
        """

        raise NotImplementedError

    # ------ Properties -------

    @property
    def id(self):
        """
        ID of this DelegationClient

        :return: ID of this Client
        :rtype: int
        """

        return self._client_id

    @property
    def active_delegations(self):
        """
        List of the IDs of all currently active delegations of this Client

        :return: List of the IDs of the delegations
        :rtype: list(int)
        """

        return self._active_delegations

"""
The DelegationManager, the core of the DelegationModule

@author: Mengers
"""

import rospy
from delegation_module.msg import CFP
from delegation_module.srv import Precommit, PrecommitResponse, \
    Propose, ProposeResponse, Failure, FailureResponse, Get_DepthResponse, Get_Depth
from delegation_errors import DelegationServiceError, DelegationPlanningWarning, DelegationContractorError, DelegationError
from delegation import Delegation, Proposal
from task import Task
from goal_wrappers import GoalWrapperBase
from delegation_clients import DelegationClientBase
from cost_evaluators import CostEvaluatorBase
from dynamic_reconfigure.server import Server as ReconfigureServer
from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure import encoding
from delegation_module.cfg import DelegationManagerConfig


class DelegationManager(object):
    """
    This class represents the manager for all delegations of this agent.
    It only communicates with other instances of the DelegationManager,
    that represent other agents, and different DelegationClients at this agent.
    It handles possible delegations from taking the goal, that could be
    delegated, to making sure a delegated task is accomplished.

    The class contains the suffixes for services and the name of the CFP-topic.
    These have to be the same for all instances of the class
    (in different processes and machines).
    """

    precom_suffix = "/precom"
    propose_suffix = "/propose"
    failure_suffix = "/failure"
    get_depth_suffix = "/get_depth"
    param_suffix = "/delegation_module_parameters"
    cfp_topic_name = "CFP_Topic"

    dynamic_reconfigure_server = None

    # should be configured according to system specs
    SERVICE_TIMEOUT = 2     # in seconds
    MAX_CONSECUTIVE_TIMEOUTS = 2    # in system steps
    MAX_CONSECUTIVE_TRIES = 10

    # can be configured to prevent loops of delegations or to deep delegations
    # NO DELEGATION will be done by the corresponding instance if this is reached
    # Set this to -1 if no boundary is needed
    MAX_DELEGATION_DEPTH = 5
    DEFAULT_AUCTION_STEPS = 1

    REPLY_IMPOSSIBLE_PROPOSALS = False

    # ------ Initiation and Config ------

    def __init__(self, name=""):
        """
        Constructor for the DelegationManager

        :param name: name of this instance of the DelegationManager,
                should be unique
        :type name: str
        """

        self._name = name

        self._param_prefix = self._name + DelegationManager.param_suffix

        self._delegations = []
        self._auction_id = 0

        self._max_tasks = 10   # should be changed by for dynamic_reconfigure
        self._tasks = []

        self._current_delegation_depth = 0

        self._cost_computable = False
        self._cost_function_evaluator = None
        self._registered_agent_name = ""
        self._manager_client_id = -1    # ID of the client of the manager
        self._active_client_ids = []   # list of client IDs

        # not started at construction
        self._get_depth_service = None

        self._init_topics()
        self._init_services()

        if not DelegationManager.dynamic_reconfigure_server:  # only one server per node
            DelegationManager.dynamic_reconfigure_server = ReconfigureServer(DelegationManagerConfig,
                                                                             self._dynamic_reconfigure_callback,
                                                                             namespace="/" + self._param_prefix)
        else:
            self._config_subscriber = rospy.Subscriber(DelegationManager.dynamic_reconfigure_server.ns
                                                       + 'parameter_updates',
                                                       ConfigMsg,
                                                       self._dynamic_reconfigure_listener_callback)

        self._loginfo("Initiation of DelegationManager completed")

    def _init_topics(self):
        """
        Initiates needed subscribers and publishers
        """

        self._cfp_publisher = rospy.Publisher(name=self.cfp_topic_name,
                                              data_class=CFP,
                                              queue_size=10)
        self._cfp_subscriber = rospy.Subscriber(name=self.cfp_topic_name,
                                                data_class=CFP,
                                                callback=self._cfp_callback)

    def _init_services(self):
        """
        Initiates needed services
        """

        self._precom_service = rospy.Service(name=self._name+self.precom_suffix,
                                             service_class=Precommit,
                                             handler=self._precom_callback)
        self._propose_service = rospy.Service(name=self._name+self.propose_suffix,
                                              service_class=Propose,
                                              handler=self._propose_callback)
        self._failure_service = rospy.Service(name=self._name+self.failure_suffix,
                                              service_class=Failure,
                                              handler=self._failure_callback)

        # not started at construction
        self._get_depth_service = None

    def update_config(self, config):
        """
        Updates configuration for this DelegationManager and the CostEvaluators
        on this node

        :param config: dict with parameters
        :type config: dict
        """

        self.MAX_CONSECUTIVE_TIMEOUTS = config.get("max_consecutive_timeouts", self.MAX_CONSECUTIVE_TIMEOUTS)
        self.MAX_CONSECUTIVE_TRIES = config.get("max_consecutive_tries", self.MAX_CONSECUTIVE_TRIES)
        self.MAX_DELEGATION_DEPTH = config.get("max_delegation_depth", self.MAX_DELEGATION_DEPTH)
        self.DEFAULT_AUCTION_STEPS = config.get("auction_steps", self.DEFAULT_AUCTION_STEPS)
        self.REPLY_IMPOSSIBLE_PROPOSALS = config.get("reply_impossible_proposals", self.REPLY_IMPOSSIBLE_PROPOSALS)
        self._max_tasks = config.get("max_tasks", self._max_tasks)

        self._loginfo("Parameters updated:" +
                      "\n\tMAX_CONSECUTIVE_TIMEOUTS\t" + str(self.MAX_CONSECUTIVE_TIMEOUTS) +
                      "\n\tMAX_CONSECUTIVE_TRIES\t" + str(self.MAX_CONSECUTIVE_TRIES) +
                      "\n\tMAX_DELEGATION_DEPTH\t" + str(self.MAX_DELEGATION_DEPTH) +
                      "\n\tAUCTION_STEPS\t\t" + str(self.DEFAULT_AUCTION_STEPS) +
                      "\n\tMAX_TASKS\t\t" + str(self._max_tasks))

        self._loginfo(CostEvaluatorBase.update_config(**config))

    # ------ Deletion methods ------

    def __del__(self):
        """
        Destructor for the DelegationManager, stops topics and services
        """

        # fail all current tasks
        for task in self._tasks:
            self.fail_task(goal_name=task.goal_name)
        self._logwarn("Stopping services and topics")
        self._stop_services()
        self._stop_topics()
        self._unregister_at_clients()

        del self._delegations[:]

    def _stop_services(self):
        """
        Shuts all services down
        """

        self._precom_service.shutdown()
        self._propose_service.shutdown()
        self._failure_service.shutdown()
        if self._get_depth_service is not None:
            self._get_depth_service.shutdown()

    def _stop_topics(self):
        """
        Unregisters publisher and subscriber of this node
        """

        self._cfp_publisher.unregister()
        self._cfp_subscriber.unregister()

    def _unregister_at_clients(self):
        """
        Unregisters this DelegationManager at all clients he is registered at
        """

        DelegationClientBase.unregister_at(self._active_client_ids)
        del self._active_client_ids[:]

    # ------ Logging Functions ------

    def _loginfo(self, string):
        """
        Rospy info logging with information about who is logging

        :param string: Message to log
        :type string: str
        """

        rospy.loginfo(str(self._name) + ": " + str(string))

    def _logwarn(self, string):
        """
        Rospy warn logging with information about who is logging

        :param string: Message to log
        :type string: str
        """

        rospy.logwarn(str(self._name) + ": " + str(string))

    # ------ Callback functions ------

    def _dynamic_reconfigure_listener_callback(self, config_msg):
        """
        Callback for the dynamic_reconfigure update message

        :param config_msg: msg
        :type config_msg: DelegationManagerConfig_msg
        """

        config = encoding.decode_config(msg=config_msg)

        self.update_config(config=config)

    def _dynamic_reconfigure_callback(self, config, _):
        """
        Direct callback of the dynamic_reconfigure server

        Level no currently used

        :param config: new config
        :type config: dict
        :return: adjusted config
        :rtype: dict
        """

        self.update_config(config=config)

        return config

    def _get_depth_callback(self, _):
        """
        Returns the current delegation depth of this DelegationManager

        Request is empty

        :return: current depth of this DelegationManager
        :rtype: Get_DepthResponse
        """

        response = Get_DepthResponse()
        response.depth = self._current_delegation_depth
        return response

    def _precom_callback(self, request):
        """
        Callback for precommit service call

        Checks if formerly proposed bid is still accurate and if it should
        make a new bid. If the bid is still accurate the DelegationManager is
        accepting the task.

        :param request: request of the Precommit.srv type
        :type request: PrecommitRequest
        :return: response to confirm or take back the old bid and possibly
                sending a new bid
        :rtype: PrecommitResponse
        """

        auctioneer_name = request.name
        auction_id = request.auction_id
        goal_representation = request.goal_representation
        goal_name = request.goal_name
        depth = request.depth
        members = request.current_members

        self._loginfo(str(auctioneer_name) + " sent a precommit for his auction "
                      + str(auction_id))

        # response with initial values, all negative
        response = PrecommitResponse()
        response.acceptance = False
        response.still_biding = False
        response.new_proposal = 0
        response.manager_name = ""

        if not self.check_possible_tasks:
            # not bidding if no new task possible right now or in general
            self._loginfo("Wont bid, because i already have enough tasks or cannot take any")
            return response

        if not self._cost_computable:
            # no cost computation available
            self._loginfo("Wont bid, because i cannot compute the cost")
            return response

        new_cost, goal_possible = self._determine_cost_and_possibility(goal_representation=goal_representation,
                                                                       depth=depth,
                                                                       members=members)

        if not goal_possible:
            self._loginfo("Earlier proposal can not be verified, goal currently not possible")
            return response

        if new_cost <= request.old_proposal:
            self._loginfo("Have accepted a contract from " + str(auctioneer_name))

            new_task = Task(auction_id=auction_id,
                            auctioneer_name=auctioneer_name,
                            goal_name=goal_name, depth=depth)

            try:
                self.add_task(new_task)
            except DelegationError as e:
                self._logwarn("Failed to add a new Task: " + str(e))
                return response

            response.acceptance = True
            response.manager_name = self._registered_agent_name
        else:
            self._loginfo("Earlier proposed cost is lower than new cost:"
                          + str(request.old_proposal) + "<" + str(new_cost))

        response.still_biding = True
        response.new_proposal = new_cost
        return response

    def _propose_callback(self, request):
        """
        Callback for propose service call

        Adds proposal to list of proposals

        :param request: request of the Propose.srv type
        :type request: ProposeRequest
        :return: empty response
        :rtype: ProposeResponse
        """

        bidder_name = request.name
        auction_id = request.auction_id
        proposed_value = request.value

        self._loginfo(str(bidder_name) + " proposed for my auction "
                      + str(auction_id) + " with " + str(proposed_value))

        try:
            delegation = self.get_delegation(auction_id=auction_id)
        except LookupError:
            self._logwarn("Proposal for not existing delegation")
            return ProposeResponse()

        new_proposal = Proposal(name=bidder_name, value=proposed_value)

        try:
            delegation.add_proposal(proposal=new_proposal)
        except Warning:
            self._loginfo("Proposal from " + str(bidder_name)
                          + " wont be added, he is forbidden")

        return ProposeResponse()

    def _failure_callback(self, request):
        """
        Callback for failure service call

        Tries to make a new auction, because the old contractor failed to
        accomplish the task

        :param request: request of the Failure.srv type
        :type request: FailureRequest
        :return: empty response
        :rtype: FailureResponse
        """

        contractor_name = request.name
        auction_id = request.auction_id
        self._loginfo(str(contractor_name) + " reported a FAILURE for my auction "
                      + str(auction_id))

        response = FailureResponse()

        try:
            delegation = self.get_delegation(auction_id=auction_id)
        except LookupError:
            self._logwarn("Failure Message for nonexistent delegation")
            return response

        try:
            delegation.get_contractor()
        except DelegationContractorError:
            self._logwarn("Failure Message for delegation without contractor")
            return response

        if delegation.get_contractor() != contractor_name:
            self._logwarn("Failure Message from source who is not its contractor")
            return response

        if delegation.check_if_goal_finished():
            self._loginfo("Delegation with ID " + str(delegation.auction_id) + " was successful")
            delegation.finish_delegation()
            client = DelegationClientBase.get_client(client_id=delegation.client_id)
            client.delegation_successful(delegation_id=delegation.auction_id)
            return response

        # delegation failed really and we have to try to find a new contractor
        delegation.fail_current_delegation()    # unregisters goal
        self._start_auction(delegation)
        return response

    def _cfp_callback(self, msg):
        """
        Callback for messages in the CFP topic, defined in self._topic_name

        Determines if a proposal for this CFP should be made and makes one
        if appropriate

        :param msg: message of the CFP.msg type
        :type msg: CFP
        """

        auctioneer_name = msg.name
        auction_id = msg.auction_id
        goal_representation = msg.goal_representation
        depth = msg.depth
        members = msg.current_members

        if auctioneer_name == self._name:
            # Do not bid this way for own auctions!
            # Give information about own capabilities at auction start
            return

        self._loginfo("Got CFP from " + str(auctioneer_name) + " with ID " + str(auction_id))

        if not self.check_possible_tasks:
            # not bidding if i cannot take tasks right now or in general
            self._loginfo("Wont bid, because i already have enough tasks or cannot take any")
            return

        if not self._cost_computable:
            # no cost computation available
            self._loginfo("Wont bid, because i cannot compute the cost")
            return

        cost, goal_possible = self._determine_cost_and_possibility(goal_representation=goal_representation,
                                                                   depth=depth,
                                                                   members=members)

        if goal_possible or self.REPLY_IMPOSSIBLE_PROPOSALS:
            self._loginfo("Sending a proposal of " + str(cost))
            try:
                self._send_propose(cost, auctioneer_name, auction_id)
            except DelegationServiceError as e:
                # Sending of a Proposal is not ultimately important
                self._logwarn("Sending of Proposal not working, continuing without (error_message:\""
                              + str(e.message) + "\")")

    # ------ Message sending methods ------

    def _send_propose(self, value, target_name, auction_id):
        """
        Sends a Propose service_name call

        :param value: proposed cost
        :type value: float
        :param target_name: name of the auctioneer
        :type target_name: str
        :param auction_id: ID of the auction
        :type auction_id: int
        :raises DelegationServiceError: if call failed
        """

        self._loginfo("Sending a proposal to " + str(target_name) + " for his auction " + str(auction_id))

        service_name = target_name + self.propose_suffix
        try:
            rospy.wait_for_service(service=service_name, timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self._logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_proposal = rospy.ServiceProxy(
                service_name, Propose)
            send_proposal(self._name, auction_id, value)
        except rospy.ServiceException:
            self._logwarn("Propose call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

    def _send_precom(self, target_name, auction_id, proposal_value,
                     goal_representation, goal_name, depth, delegation_members):
        """
        Calls the Precommit service of the winning bidder of this delegation

        :param target_name: name of the bidder who gets the precom
        :type target_name: str
        :param auction_id: ID of the corresponding auction
        :type auction_id: int
        :param proposal_value: proposed value
        :type proposal_value: float
        :param goal_representation: representation of the goal for this auction
        :type goal_representation: str
        :param goal_name: name of the goal
        :type goal_name: str
        :param depth: current depth of this delegation
        :type depth: int
        :param delegation_members: list of current members of the delegation
        :type delegation_members: list(str)
        :return: response of the service call,
                includes the acceptance of the bidder or possibly a new proposal
        :rtype: PrecommitResponse
        :raises DelegationServiceError: if call failed
        """

        self._loginfo("Sending a precommit to " + str(target_name) + " for my auction " + str(auction_id))

        service_name = target_name + self.precom_suffix
        try:
            rospy.wait_for_service(service=service_name, timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self._logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_precom = rospy.ServiceProxy(service_name, Precommit)
            response = send_precom(goal_representation, self._name, goal_name,
                                   auction_id, proposal_value,
                                   depth, delegation_members)
        except rospy.ServiceException:
            self._logwarn("Precommit call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

        return response

    def _send_failure(self, auctioneer_name, auction_id):
        """
        Sends a Failure service-call for the specified name, id

        :param auctioneer_name: name of the auctioneer of the failed task
        :type auctioneer_name: str
        :param auction_id: ID of the failed task
        :type auction_id: int
        :raises DelegationServiceError: if call failed
        """

        self._loginfo("Sending a Failure message to " + str(auctioneer_name)
                      + " for his auction " + str(auction_id))

        service_name = auctioneer_name + self.failure_suffix
        try:
            rospy.wait_for_service(service=service_name, timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self._logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_failure = rospy.ServiceProxy(service_name, Failure)
            send_failure(self._name, auction_id)
        except rospy.ServiceException:
            self._logwarn("Failure call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

    def _send_cfp(self, goal_representation, auction_id, depth, delegation_members):
        """
        Sends a CFP-broadcast over the CFP-topic

        :param goal_representation: goal_information
        :type goal_representation: str
        :param auction_id: ID of the corresponding auction
        :type auction_id: int
        :param depth: depth of this delegation
        :type depth: int
        :param delegation_members: List of current Members of the Delegation
        :type delegation_members: list(str)
        :raises ServiceException: if call failed
        """

        self._loginfo("Sending a CFP for my auction " + str(auction_id))

        msg = CFP()
        msg.goal_representation = goal_representation
        msg.name = self._name
        msg.auction_id = auction_id
        msg.depth = depth
        msg.current_members = delegation_members

        try:
            self._cfp_publisher.publish(msg)
        except rospy.ROSException:
            self._logwarn("CFP publish failed")
            raise DelegationServiceError("Call failed: CFP")

    def _send_get_depth(self, prefix):
        """
        Sends a Get_Depth service call to the given agent

        :param prefix: prefix for this call
        :type prefix: str
        :return: service-response
        :rtype: Get_DepthResponse
        :raises DelegationServiceError: if call unsuccessful
        """

        self._loginfo("Sending a Get_Depth request to " + str(prefix))

        service_name = prefix + self.get_depth_suffix
        try:
            rospy.wait_for_service(service=service_name, timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self._logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_get_depth = rospy.ServiceProxy(service_name, Get_Depth)
            depth = send_get_depth()
            return depth
        except rospy.ServiceException:
            self._logwarn("Get_Depth call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

    # ---- Depths ----

    def check_remote_depth(self, prefix):
        """
        Gets the depth from a remote source with the given service-agent_name

        :param prefix: service-prefix of source from which the depth is to be
                determined
        :type prefix: str
        :return: depth if call successful, else None
        :rtype: int or None
        """

        try:
            response = self._send_get_depth(prefix=prefix)
            depth = response.depth
        except DelegationServiceError:
            self._logwarn("Could not determine the remote depth for " + str(prefix) + " --> will return None")
            depth = None
        return depth

    def start_depth_service(self, prefix):
        """
        Starts the Get_Depth service for this Manager with the given prefix

        :param prefix: prefix that should be used, e.g. the name of this agent
        :type prefix: str
        """

        name = prefix + self.get_depth_suffix
        self._get_depth_service = rospy.Service(name=name, service_class=Get_Depth, handler=self._get_depth_callback)

    def stop_depth_service(self):
        """
        Stop the Get_Depth service of this DelegationManager
        """

        if self._get_depth_service is not None:
            self._get_depth_service.shutdown()
            self._get_depth_service = None

    def update_delegation_depth(self):
        """
        Updates the depth of this DelegationManager according to current tasks
        """

        if len(self._tasks) == 0:
            self._current_delegation_depth = 0
        else:
            self._current_delegation_depth = max([t.depth for t in self._tasks])

    # ---- Clients ----

    def add_client(self, client_id):
        """
        Adds the client with the given ID to this DelegationManager

        :param client_id: ID of the client
        :type client_id: int
        """

        self._active_client_ids.append(client_id)

    def remove_client(self, client_id):
        """
        Removes the client with the given ID

        :param client_id: ID of the client
        :type client_id: int
        """

        if self._active_client_ids.__contains__(client_id):
            self._active_client_ids.remove(client_id)

    # ---- Cost Function Evaluator ----

    def set_cost_function_evaluator(self, cost_function_evaluator, agent_name, client_id):
        """
        Adds a cost_function_evaluator, overwrites old evaluator if there is
        one and makes it possible for the DelegationManager to compute the cost
        of tasks with this evaluator

        :param cost_function_evaluator: a working cost_function_evaluator
        :type cost_function_evaluator: CostEvaluatorBase
        :param agent_name: name of the manager the evaluator is from
        :type agent_name: str
        :param client_id: ID of the client at this manager
        :type client_id: int
        """

        self._manager_client_id = client_id
        self._cost_function_evaluator = cost_function_evaluator
        self._cost_computable = True
        self._registered_agent_name = agent_name

    def remove_cost_function_evaluator(self):
        """
        Deletes currently used cost_function_evaluator and makes it impossible
        to compute cost of tasks
        """

        self._cost_computable = False
        del self._cost_function_evaluator
        self._cost_function_evaluator = None

    # ------ Tasks, Delegations, IDs ------

    def add_task(self, new_task):
        """
        Adds the task to the task list if possible

        :param new_task: the new task
        :type new_task: Task
        :raises DelegationError: if it cant be added
        """

        if len(self._tasks) >= self._max_tasks != -1:
            raise DelegationError

        self._tasks.append(new_task)

        self.update_delegation_depth()

    def get_task_by_goal_name(self, goal_name):
        """
        Returns a currently running task with the given goal

        :param goal_name: name of the goal that the task has
        :type goal_name: str
        :return: the running task
        :rtype: Task
        :raises LookupError: if there is no task with that goal
        """

        for task in self._tasks:
            if task.goal_name == goal_name:
                return task

        raise LookupError("No task with the goal named " + str(goal_name))

    def get_delegation(self, auction_id):
        """
        Gets the delegation with this auction_id or raises an exception if
        there is no delegation with this id

        :param auction_id: auction_id of an existing delegation
        :type auction_id: int
        :return: the relevant delegation
        :rtype: Delegation
        :raises LookupError: if there is no delegation with this auction_id
        """

        for delegation in self._delegations:
            if delegation.auction_id == auction_id:
                return delegation

        # if no delegation with this name exists
        raise LookupError("No delegation with the auction_id " + str(auction_id))

    def get_new_auction_id(self):
        """
        Creates a new auction_id for this instance of the DelegationManager

        :return: The new auction_id
        :rtype: int
        """

        self._auction_id += 1

        # check that the auction_id is usable in ROS msgs/services
        uint32_max = 2**32 - 1
        if self._auction_id > uint32_max:
            self._logwarn("Space for auction IDs is exhausted,"
                          " starting with 0 again. " 
                          "This could possibly lead to problems!")
            self._auction_id = 0

        return self._auction_id

    # ------ Make auctions etc, internal methods ------

    def _determine_cost_and_possibility(self, goal_representation, depth, members):
        """
        Determines cost and possibility using the CostEvaluator

        :param goal_representation: representation of the goal
        :type goal_representation: str
        :param depth: depth of the goal
        :type depth: int
        :param members: list of the current members of the delegation
        :type members: list(str)
        :return: Cost and Possibility of the goal
        :rtype: float, bool
        """

        try:
            cost, goal_possible = self._cost_function_evaluator.compute_cost_and_possibility(goal_representation=goal_representation,
                                                                                             current_task_count=len(self._tasks),
                                                                                             max_task_count=self._max_tasks,
                                                                                             current_depth=depth,
                                                                                             max_depth=self.MAX_DELEGATION_DEPTH,
                                                                                             members=members,
                                                                                             own_name=self._name)
            if goal_possible:
                s = "possible with a cost of " + str(cost)
            else:
                s = "impossible"
            self._loginfo("Task is " + s)
        except DelegationPlanningWarning as e:
            self._loginfo("Goal not possible. PlannerMessage: " + str(e.message))
            cost, goal_possible = CostEvaluatorBase.IMPOSSIBLE_COSTS, False
        return cost, goal_possible

    def _check_depth(self, depth):
        """
        Checks whether the given delegation depth makes a delegation possible
        or not.

        :param depth: depth that has to be checked
        :type depth: int
        :return: whether it is possible or not
        :rtype: bool
        """

        if self.MAX_DELEGATION_DEPTH == -1:
            return True
        if depth < self.MAX_DELEGATION_DEPTH:
            return True

        self._logwarn("Maximum Delegation Depth is reached")
        return False

    def _precom(self, proposal, delegation):
        """
        Sends a precom call for the delegation with this proposal

        Wrapper for the call

        :param proposal: the proposal that won
        :type proposal: Proposal
        :param delegation: the delegation with this proposal
        :type delegation: Delegation
        :return: response of the call
        :rtype: PrecommitResponse
        """

        if self.depth_checking_possible:
            depth = self._current_delegation_depth + 1
        else:
            depth = delegation.depth + 1

        employers = self.current_employers
        employers.append(self._name)
        delegation_members = list(set(employers))

        response = self._send_precom(target_name=proposal.name,
                                     auction_id=delegation.auction_id,
                                     goal_name=delegation.get_goal_name(),
                                     proposal_value=proposal.value,
                                     goal_representation=delegation.get_goal_representation(),
                                     delegation_members=delegation_members,
                                     depth=depth)
        return response

    def _start_auction(self, delegation):
        """
        Starts the auction for this delegation with a CFP message

        :param delegation: the delegation for which the auction should be
                started
        :type delegation: Delegation
        """

        # Making sure that the delegation is in the right state
        delegation.start_auction()

        if self.depth_checking_possible:
            depth = self._current_delegation_depth + 1
        else:
            depth = delegation.depth + 1

        auction_id = delegation.auction_id
        goal_representation = delegation.get_goal_representation()

        employers = self.current_employers
        employers.append(self._name)
        delegation_members = list(set(employers))

        self._loginfo("Starting auction with ID: " + str(auction_id))
        try:
            self._send_cfp(goal_representation=goal_representation,
                           auction_id=auction_id,
                           depth=depth,
                           delegation_members=delegation_members)
        except DelegationServiceError as e:
            self._logwarn("CFP was not sent right (error_message:\"" + str(e.message) + "\")")
            # restart auction next step by ending it than (it will restart,
            # because it has no valid proposals than)
            delegation.end_auction_next_step()

    def _end_auction(self, delegation):
        """
        Stops the auction for the given delegation, determines its winner and
        tries to make him the contractor

        :param delegation: the delegation of which the auction should end
        :type delegation: Delegation
        :return: whether it was successfully ended or not (restarted)
        :rtype: bool
        """

        auction_id = delegation.auction_id
        self._loginfo("Trying to end auction with ID " + str(auction_id))

        up_for_delegation = True

        for counter in range(self.MAX_CONSECUTIVE_TRIES):

            if not delegation.has_proposals():
                self._logwarn("Auction with ID " + str(auction_id)
                              + " has no proposals")
                break

            try:
                best_proposal = delegation.get_best_proposal()
            except LookupError:
                self._logwarn("Auction with ID " + str(auction_id)
                              + " has no proposals")
                break

            bidder_name = best_proposal.name
            proposed_value = best_proposal.value

            if bidder_name == self._name:
                self._loginfo("I won my own auction with the ID "
                              + str(auction_id))
                # make sure i do the work myself
                client_id = delegation.client_id
                client = DelegationClientBase.get_client(client_id=client_id)
                client.start_work_for_delegation(delegation_id=delegation.auction_id)
                up_for_delegation = False
                break

            self._loginfo("Sending a precommit to " + str(bidder_name) +
                          " who bid " + str(proposed_value) +
                          " for my auction " + str(auction_id))

            try:
                response = self._precom(proposal=best_proposal, delegation=delegation)
            except DelegationServiceError as e:
                # if the best bid is not reachable, try next best bid, instead of giving up auction
                self._logwarn("Precommit failed, trying next best Bidder if applicable (error_message:\""
                              + str(e.message) + "\")")
                delegation.remove_proposal(proposal=best_proposal)
                continue

            manager_name = response.manager_name

            if response.acceptance:
                self._loginfo(str(bidder_name) + " has accepted the contract for a cost of "
                              + str(proposed_value) + " for my auction " + str(auction_id))

                try:
                    # set contractor, change delegation state and send goal
                    delegation.make_contract(bidder_name=bidder_name, manager_name=manager_name)
                except DelegationContractorError:
                    self._logwarn("Contractor has already been chosen, while i am trying to find one")
                    up_for_delegation = False
                    break
                except DelegationError as e:
                    self._logwarn("Sending goal was not possible! (error_message:\""
                                  + str(e.message) + "\")")

                    # make sure goal is not living anymore and contractor is removed
                    delegation.terminate_contract()

                    delegation.remove_proposal(proposal=best_proposal)
                    continue

                # Contractor has been found
                up_for_delegation = False
                break

            elif response.still_biding:
                self._loginfo(str(bidder_name) + " has given a new proposal of " + str(response.new_proposal)
                              + " for my auction " + str(auction_id))
                delegation.remove_proposal(proposal=best_proposal)

                if best_proposal.value >= response.new_proposal:
                    self._logwarn("The new proposal is not worse than the old proposal while he is not accepting the old proposal,"
                                  + " something is off!\nWont add this new proposal just to be safe")
                    continue

                try:
                    delegation.add_proposal(proposal=Proposal(bidder_name, response.new_proposal))
                except Warning:
                    self._loginfo("Proposal from " + bidder_name + " wont be added, he is forbidden")

            else:
                self._loginfo(str(bidder_name) + " has stopped biding for my auction " + str(auction_id))
                delegation.remove_proposal(proposal=best_proposal)

        if up_for_delegation:
            self._logwarn("No possible contractor has been found for my auction " + str(auction_id))
            # starting a new delegation (real failure is currently not possible
            self._start_auction(delegation=delegation)
            return False
        else:
            self._loginfo("Auction with ID " + str(delegation.auction_id) + " is finished")
            return True

    # ------ Functions to interact with the DelegationManager ------

    def terminate(self, auction_id):
        """
        Ends a delegation and does not try to delegate the task again

        :param auction_id: the id of the delegation that should be terminated
        :type auction_id: int
        """

        try:
            delegation = self.get_delegation(auction_id=auction_id)
        except LookupError:
            self._loginfo("Trying to terminate delegation with the auction id "
                          + str(auction_id) + ", that doesnt exist")
            return

        delegation.finish_delegation()

    def fail_task(self, goal_name):
        """
        Sends a message to the associated employer, that this task will no
        longer be pursued, if this is a task that comes from a different source
        Will not do anything if the goal is not given via this DelegationManager

        :param goal_name: name of the goal that failed
        :type goal_name: str
        """

        try:
            task = self.get_task_by_goal_name(goal_name=goal_name)
        except LookupError:
            # this goal was not given by a different manager as task, do nothing
            return

        # send a failure right, if needed
        try:
            self._send_failure(auctioneer_name=task.auctioneer_name, auction_id=task.auction_id)
        except DelegationServiceError as e:
            rospy.logerr("Failure Message raised following error: "+e.message)
            pass

    def delegate(self, goal_wrapper, client_id, auction_steps=None, own_cost=CostEvaluatorBase.IMPOSSIBLE_COSTS,
                 known_depth=None):
        """
        Makes a delegation for the goal and starts an auction for this
        delegation. Adds my own cost as a proposal if wanted.

        The auction will be closed after the given number of steps were taken,
        a winner will be determined and the goal will be delegated to that
        winner.

        :param goal_wrapper: wrapper for the goal that should be delegated
        :type goal_wrapper: GoalWrapperBase
        :param client_id: ID of the client who starts this delegation
        :type client_id: int
        :param auction_steps: number of steps
                that are waited for proposals while the auction is running
        :type auction_steps: int
        :param own_cost: cost if i have to achieve the goal myself, greater than
                0 if not achievable for me
        :type own_cost: int
        :param known_depth: if a specific delegation depth is known for this
                delegation, give this depth here
        :type known_depth: int
        :return: the auction_id of the auction
        :rtype: int
        :raises DelegationError: if the MAX_DELEGATION_DEPTH is reached
        """

        if auction_steps is None:
            auction_steps = self.DEFAULT_AUCTION_STEPS

        if known_depth is not None:
            depth = known_depth
        elif self.depth_checking_possible:
            depth = self._current_delegation_depth
        else:
            depth = -1

        if not self._check_depth(depth=depth):
            raise DelegationError("MAX_DELEGATION_DEPTH is reached")

        new = Delegation(goal_wrapper=goal_wrapper, auction_id=self.get_new_auction_id(),
                         auction_steps=auction_steps, client_id=client_id, depth=depth,
                         max_timeout_steps=self.MAX_CONSECUTIVE_TIMEOUTS)

        self._delegations.append(new)
        self._start_auction(delegation=new)

        if own_cost > 0:
            proposal = Proposal(name=self._name, value=own_cost)
            new.add_proposal(proposal=proposal)

        return new.auction_id

    def do_step(self, delegation_ids):
        """
        Performs a step for all delegations with a given Id

        :param delegation_ids: list of IDs of delegations that should be stepped
        :type delegation_ids: list(int)
        """

        if len(delegation_ids) == 0:
            return

        self._loginfo("Doing a step for auctions: " + delegation_ids.__repr__())

        delegations = [self.get_delegation(auction_id=i) for i in delegation_ids]
        waiting_delegations = [d for d in delegations if d.state.is_waiting_for_proposals()]
        running_delegations = [d for d in delegations if d.state.is_delegated_running()]

        for delegation in running_delegations:
            if not delegation.check_if_alive():
                self._logwarn("Too many TIMEOUTS for the contractor \"" + str(delegation.get_contractor())
                              + "\" of the auction with the ID " + str(delegation.auction_id)
                              + "! Will try to find a new contractor.")
                delegation.fail_current_delegation()  # unregisters goal
                self._start_auction(delegation)

            if delegation.check_if_goal_finished():
                self._loginfo("Delegation with ID " + str(delegation.auction_id) + " was successful")
                delegation.finish_delegation()
                client = DelegationClientBase.get_client(client_id=delegation.client_id)
                client.delegation_successful(delegation_id=delegation.auction_id)

        for delegation in waiting_delegations:
            # decrementing the needed steps and checking at the same time
            if delegation.decrement_and_check_steps():
                self._end_auction(delegation=delegation)

    def end_task(self, goal_name):
        """
        Tries to end the task with a goal with this name

        :param goal_name: name of the goal that is no longer a task
        :type goal_name: str
        """

        try:
            task = self.get_task_by_goal_name(goal_name=goal_name)
            self._tasks.remove(task)
            self.update_delegation_depth()
            self._loginfo("Task with goal_name " + goal_name + " was finished")
            del task
        except LookupError:
            # this goal was no task given by a different manager
            return

    # ------ Properties ------

    @property
    def name(self):
        """
        Gets the DelegationManager name

        :return: name of this delegation manager
        :rtype: str
        """

        return self._name

    @property
    def current_employers(self):
        """
        List of current Employers of this DelegationManager according to the
        current tasks

        :return: list of the employers
        :rtype: list(str)
        """

        employers = list()
        for t in self._tasks:
            employers.extend(t.employers)

        return list(set(employers))

    @property
    def depth_checking_possible(self):
        """
        Whether Depth checking of this DelegationManager is currently possible

        :return: whether Depth checking is possible
        :rtype: bool
        """

        # is only given if cost computable
        return self._cost_computable

    @property
    def cost_computable(self):
        """
        Whether Cost is currently computable at this DelegationManager

        :return: whether Cost is currently computable at this DelegationManager
        :rtype: bool
        """

        return self._cost_computable

    @property
    def check_possible_tasks(self):
        """
        Whether additional tasks are possible right now or not

        :return: whether additional tasks are possible right now
        :rtype: bool
        """

        if len(self._tasks) >= self._max_tasks != -1:
            return False
        else:
            return True

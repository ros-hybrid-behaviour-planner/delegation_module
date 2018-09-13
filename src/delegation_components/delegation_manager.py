
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
    that represent other agents, and the general manager of its own agent.
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

    # ------ Initiation methods ------

    def __init__(self, instance_name="", max_tasks=0):
        """
        Constructor for the DelegationManager

        :param instance_name: name of this instance of the DelegationManager,
                should be unique
        :type instance_name: str
        :param max_tasks: number of maximum tasks that can simultaneously run,
                set this to 0 for no possible tasks or -1 for unlimited number
                of tasks
        :type max_tasks: int
        """

        self._name = instance_name

        self._param_prefix = self._name + DelegationManager.param_suffix

        self.__delegations = []
        self.__auction_id = 0

        self.__max_tasks = max_tasks
        self.__tasks = []

        self.__current_delegation_depth = 0

        self.__cost_computable = False
        self.__cost_function_evaluator = None
        self.__registered_manager = ""
        self.__manager_client_id = 0    # ID of the client of the manager
        self.__active_client_ids = []   # list of client IDs

        # not started at construction
        self._get_depth_service = None

        self.__init_topics()
        self.__init_services()

        if not DelegationManager.dynamic_reconfigure_server:  # only one server per node
            DelegationManager.dynamic_reconfigure_server = ReconfigureServer(DelegationManagerConfig, self._dynamic_reconfigure_callback,
                                                                             namespace="/" + self._param_prefix)
        else:
            self.__config_subscriber = rospy.Subscriber(DelegationManager.dynamic_reconfigure_server.ns + 'parameter_updates',
                                                        ConfigMsg, self._dynamic_reconfigure_listener_callback)

        self.__loginfo("Initiation of DelegationManager completed")

    def __init_topics(self):
        """
        Initiates needed subscribers and publishers
        """

        self._cfp_publisher = rospy.Publisher(name=self.cfp_topic_name, data_class=CFP, queue_size=10)
        self._cfp_subscriber = rospy.Subscriber(name=self.cfp_topic_name, data_class=CFP, callback=self.__cfp_callback)

    def __init_services(self):
        """
        Initiates needed services
        """

        self._precom_service = rospy.Service(name=self._name+self.precom_suffix, service_class=Precommit, handler=self.__precom_callback)
        self._propose_service = rospy.Service(name=self._name+self.propose_suffix, service_class=Propose, handler=self.__propose_callback)
        self._failure_service = rospy.Service(name=self._name+self.failure_suffix, service_class=Failure, handler=self.__failure_callback)

        # not started at construction
        self._get_depth_service = None

    # ------ Deletion methods ------

    def __del__(self):
        """
        Destructor for the DelegationManager
        """

        # fail all current tasks
        for task in self.__tasks:
            self.fail_task(goal_name=task.goal_name())
        self.__logwarn("Stopping services and topics")
        self.__stop_services()
        self.__stop_topics()
        self.__unregister_at_clients()

        del self.__delegations[:]

    def __stop_services(self):
        """
        Shuts all services down
        """

        self._precom_service.shutdown()
        self._propose_service.shutdown()
        self._failure_service.shutdown()
        if self._get_depth_service is not None:
            self._get_depth_service.shutdown()

    def __stop_topics(self):
        """
        Unregisters publisher and subscriber of this node
        """

        self._cfp_publisher.unregister()
        self._cfp_subscriber.unregister()

    def __unregister_at_clients(self):
        """
        Unregisters this DelegationManager at all clients he is registered at
        """

        DelegationClientBase.unregister_at(self.__active_client_ids)
        del self.__active_client_ids[:]

    # ------ Logging Functions ------

    def __loginfo(self, string):
        """
        Rospy info logging with information about who is logging

        :param string: Message to log
        :type string: str
        """

        rospy.loginfo(str(self._name) + ": " + str(string))

    def __logwarn(self, string):
        """
        Rospy warn logging with information about who is logging

        :param string: Message to log
        :type string: str
        """

        rospy.logwarn(str(self._name) + ": " + str(string))

    # ------ Callback functions ------

    def _dynamic_reconfigure_listener_callback(self, config_msg):
        """
        callback for the dynamic_reconfigure update message

        :param config_msg: msg
        :type config_msg: DelegationManagerConfig_msg
        """

        config = encoding.decode_config(msg=config_msg)

        self.update_config(config=config)

    # noinspection PyUnusedLocal
    def _dynamic_reconfigure_callback(self, config, level):
        """
        direct callback of the dynamic_reconfigure server

        :param config: new config
        :type config: dict
        :param level: not currently supported
        :return: adjusted config
        :rtype: dict
        """

        self.update_config(config=config)

        return config

    # noinspection PyUnusedLocal
    def __get_depth_callback(self, request):
        """
        Returns the current delegation depth of this DelegationManager

        :param request: request of the Get_Depth.srv type (empty)
        :return: response with the current delegation depth
        """

        response = Get_DepthResponse()
        response.depth = self.__current_delegation_depth
        return response

    def __precom_callback(self, request):
        """
        Callback for precommit service call

        Checks if formerly proposed bid is still accurate and if it should
        make a new bid. If the bid is still accurate the DelegationManager is
        accepting the task.

        :param request: request of the Precommit.srv type
        :return: response to confirm or take back the old bid and possibly
                sending a new bid
        """

        auctioneer_name = request.name
        auction_id = request.auction_id
        goal_representation = request.goal_representation
        goal_name = request.goal_name
        depth = request.depth
        members = request.current_members

        self.__loginfo(str(auctioneer_name) + " sent a precommit for his auction " + str(auction_id))

        # response with initial values, all negative
        response = PrecommitResponse()
        response.acceptance = False
        response.still_biding = False
        response.new_proposal = 0
        response.manager_name = ""

        if not self.check_possible_tasks():
            # not bidding if no new task possible right now or in general
            self.__loginfo("Wont bid, because i already have enough tasks or cannot take any")
            return response

        if not self.__cost_computable:
            # no cost computation available
            self.__loginfo("Wont bid, because i cannot compute the cost")
            return response

        try:
            new_cost, possible_goal = self.__cost_function_evaluator.compute_cost_and_possibility(goal_representation=goal_representation,
                                                                                                  current_task_count=len(self.__tasks),
                                                                                                  max_task_count=self.__max_tasks,
                                                                                                  current_depth=depth,
                                                                                                  max_depth=self.MAX_DELEGATION_DEPTH,
                                                                                                  members=members,
                                                                                                  own_name=self._name)
            if possible_goal:
                s = "possible with a cost of " + str(new_cost)
            else:
                s = "impossible"
            self.__loginfo("Task is " + s)
        except DelegationPlanningWarning as e:
            self.__loginfo("Goal not possible. PlannerMessage: " + str(e.message))
            new_cost, possible_goal = -1, False

        if not possible_goal:
            self.__loginfo("Earlier proposal can not be verified, goal currently not possible")
            return response

        if new_cost <= request.old_proposal:
            self.__loginfo("Have accepted a contract from " + str(auctioneer_name))

            new_task = Task(auction_id=auction_id, auctioneer_name=auctioneer_name, goal_name=goal_name, depth=depth)
            try:
                self.add_task(new_task)
            except DelegationError as e:
                self.__logwarn("Failed to add a new Task: " + str(e))
                return response

            response.acceptance = True
            response.manager_name = self.__registered_manager

        else:
            self.__loginfo("Earlier proposed cost is lower than new cost:" + str(request.old_proposal) + "<" + str(new_cost))

        response.still_biding = True
        response.new_proposal = new_cost
        return response

    def __propose_callback(self, request):
        """
        Callback for propose service call

        Adds proposal to list of proposals

        :param request: request of the Propose.srv type
        :return: empty response
        """

        bidder_name = request.name
        auction_id = request.auction_id
        proposed_value = request.value

        self.__loginfo(str(bidder_name) + " proposed for my auction " + str(auction_id) + " with " + str(proposed_value))

        try:
            delegation = self.get_delegation(auction_id=auction_id)
        except LookupError:
            self.__logwarn("Proposal for not existing delegation")
            return ProposeResponse()

        new_proposal = Proposal(name=bidder_name, value=proposed_value)

        try:
            delegation.add_proposal(proposal=new_proposal)
        except Warning:
            self.__loginfo("Proposal from " + str(bidder_name) + " wont be added, he is forbidden")

        return ProposeResponse()

    def __failure_callback(self, request):
        """
        Callback for failure service call

        Tries to make a new auction, because the old contractor failed to
        accomplish the task

        :param request: request of the Failure.srv type
        :return: empty response
        """

        contractor_name = request.name
        auction_id = request.auction_id

        self.__loginfo(str(contractor_name) + " reported a FAILURE for my auction " + str(auction_id))

        response = FailureResponse()
        try:
            delegation = self.get_delegation(auction_id=auction_id)
        except LookupError:
            self.__logwarn("Failure Message for nonexistent delegation")
            return response

        try:
            delegation.get_contractor()
        except DelegationContractorError:
            self.__logwarn("Failure Message for delegation without contractor")
            return response

        if delegation.get_contractor() != contractor_name:
            self.__logwarn("Failure Message from source who is not its contractor")
            return response

        delegation.forbid_bidder(name=contractor_name)
        delegation.fail_current_delegation()    # unregisters goal

        self.__start_auction(delegation)

        return response

    def __cfp_callback(self, msg):
        """
        Callback for messages in the CFP topic, defined in self._topic_name

        Determines if a proposal for this CFP should be made and makes one
        if appropriate

        :param msg: message of the CFP.msg type
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

        self.__loginfo("Got CFP from " + str(auctioneer_name) + " with ID " + str(auction_id))

        if not self.check_possible_tasks():
            # not bidding if i cannot take tasks right now or in general
            self.__loginfo("Wont bid, because i already have enough tasks or cannot take any")
            return

        if not self.__cost_computable:
            # no cost computation available
            self.__loginfo("Wont bid, because i cannot compute the cost")
            return

        try:
            cost, possible_goal = self.__cost_function_evaluator.compute_cost_and_possibility(goal_representation=goal_representation,
                                                                                              current_task_count=len(self.__tasks),
                                                                                              max_task_count=self.__max_tasks,
                                                                                              current_depth=depth,
                                                                                              max_depth=self.MAX_DELEGATION_DEPTH,
                                                                                              members=members,
                                                                                              own_name=self._name)
            if possible_goal:
                s = "possible with a cost of " + str(cost)
            else:
                s = "impossible"
            self.__loginfo("Task is " + s)
        except DelegationPlanningWarning as e:
            self.__loginfo("Goal not possible. PlannerMessage: " + str(e.message))
            cost, possible_goal = -1, False

        if possible_goal:
            self.__loginfo("Sending a proposal of " + str(cost))
            try:
                self.__send_propose(cost, auctioneer_name, auction_id)
            except DelegationServiceError as e:
                # Sending of a Proposal is not ultimately important
                self.__logwarn("Sending of Proposal not working, continuing without (error_message:\"" + str(e.message) + "\")")

    # ------ Message sending methods ------

    def __send_propose(self, value, target_name, auction_id):
        """
        Sends a Propose service_name call

        :param value: proposed cost
        :param target_name: name of the auctioneer
        :param auction_id: ID of the auction
        :raises DelegationServiceError: if call failed
        """

        self.__loginfo("Sending a proposal to " + str(target_name) + " for his auction " + str(auction_id))

        service_name = target_name + self.propose_suffix
        try:
            rospy.wait_for_service(service=service_name, timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_proposal = rospy.ServiceProxy(
                service_name, Propose)
            send_proposal(self._name, auction_id, value)
        except rospy.ServiceException:
            self.__logwarn("Propose call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

    def __send_precom(self, target_name, auction_id, proposal_value, goal_representation, goal_name, depth, delegation_members):
        """
        Calls the Precommit service of the winning bidder of this delegation

        :param target_name: name of the bidder who gets the precom
        :param auction_id: ID of the corresponding auction
        :param proposal_value: proposed value
        :param goal_representation: representation of the goal for this auction
        :return: response of the service call,
                includes the acceptance of the bidder or possibly a new proposal
        :raises DelegationServiceError: if call failed
        """

        self.__loginfo("Sending a precommit to " + str(target_name) + " for my auction " + str(auction_id))

        service_name = target_name + self.precom_suffix
        try:
            rospy.wait_for_service(service=service_name, timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_precom = rospy.ServiceProxy(service_name, Precommit)
            response = send_precom(goal_representation, self._name, goal_name, auction_id, proposal_value, depth, delegation_members)
        except rospy.ServiceException:
            self.__logwarn("Precommit call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

        return response

    def __send_failure(self, auctioneer_name, auction_id):
        """
        Sends a Failure service-call for the specified name, id

        :param auctioneer_name: name of the auctioneer of the failed task
        :param auction_id: ID of the failed task
        :raises DelegationServiceError: if call failed
        """

        self.__loginfo("Sending a Failure message to " + str(auctioneer_name) + " for his auction " + str(auction_id))

        service_name = auctioneer_name + self.failure_suffix
        try:
            rospy.wait_for_service(service=service_name, timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_failure = rospy.ServiceProxy(service_name, Failure)
            send_failure(self._name, auction_id)
        except rospy.ServiceException:
            self.__logwarn("Failure call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

    def __send_cfp(self, goal_representation, auction_id, depth, delegation_members):
        """
        Sends a CFP-broadcast over the CFP-topic

        :param goal_representation: goal_information
        :param auction_id: ID of the corresponding auction
        :raises ServiceException: if call failed
        """

        self.__loginfo("Sending a CFP for my auction " + str(auction_id))

        msg = CFP()
        msg.goal_representation = goal_representation
        msg.name = self._name
        msg.auction_id = auction_id
        msg.depth = depth
        msg.current_members = delegation_members

        try:
            self._cfp_publisher.publish(msg)
        except rospy.ROSException:
            self.__logwarn("CFP publish failed")
            raise DelegationServiceError("Call failed: CFP")

    def __send_get_depth(self, prefix):
        self.__loginfo("Sending a Get_Depth request to " + str(prefix))

        service_name = prefix + self.get_depth_suffix
        try:
            rospy.wait_for_service(service=service_name, timeout=self.SERVICE_TIMEOUT)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_get_depth = rospy.ServiceProxy(service_name, Get_Depth)
            depth = send_get_depth()
            return depth
        except rospy.ServiceException:
            self.__logwarn("Get_Depth call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

    # ------ Simple Getter/Setter for members ------

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
        self.__max_tasks = config.get("max_tasks", self.__max_tasks)

        self.__loginfo("Parameters updated:" +
                       "\n\tMAX_CONSECUTIVE_TIMEOUTS\t" + str(self.MAX_CONSECUTIVE_TIMEOUTS) +
                       "\n\tMAX_CONSECUTIVE_TRIES\t" + str(self.MAX_CONSECUTIVE_TRIES) +
                       "\n\tMAX_DELEGATION_DEPTH\t" + str(self.MAX_DELEGATION_DEPTH) +
                       "\n\tAUCTION_STEPS\t\t" + str(self.DEFAULT_AUCTION_STEPS) +
                       "\n\tMAX_TASKS\t\t" + str(self.__max_tasks))

        self.__loginfo(CostEvaluatorBase.update_config(**config))

    def check_remote_depth(self, prefix):
        """
        Gets the depth from a remote source with the given service-prefix

        :param prefix: service-prefix of source from which the depth is to be
                determined
        :type prefix: str
        :return: depth if call successful, else None
        :rtype: int or None
        """

        try:
            response = self.__send_get_depth(prefix=prefix)
            depth = response.depth
        except DelegationServiceError:
            self.__logwarn("Could not determine the remote depth for "+str(prefix)+" --> will return None")
            depth = None
        return depth

    def add_task(self, new_task):
        """
        Adds the task to the task list if possible

        :param new_task: the new task
        :raises DelegationError: if it cant be added
        """

        if len(self.__tasks) >= self.__max_tasks != -1:
            raise DelegationError

        self.__tasks.append(new_task)

        self.update_delegation_depth()

    def add_client(self, client_id):
        self.__active_client_ids.append(client_id)

    def update_delegation_depth(self):
        if len(self.__tasks) == 0:
            self.__current_delegation_depth = 0
        else:
            self.__current_delegation_depth = max([t.depth for t in self.__tasks])

    def remove_client(self, client_id):
        if self.__active_client_ids.__contains__(client_id):
            self.__active_client_ids.remove(client_id)

    def start_depth_service(self, prefix):
        name = prefix + self.get_depth_suffix
        self._get_depth_service = rospy.Service(name=name, service_class=Get_Depth, handler=self.__get_depth_callback)

    def stop_depth_service(self):
        if self._get_depth_service is not None:
            self._get_depth_service.shutdown()
            self._get_depth_service = None

    def set_cost_function_evaluator(self, cost_function_evaluator, manager_name, client_id):
        """
        Adds a cost_function_evaluator, overwrites old evaluator if there is
        one and makes it possible for the delegation_manager to compute the cost
        of tasks with this evaluator

        :param cost_function_evaluator: a working cost_function_evaluator
        :type cost_function_evaluator: CostEvaluatorBase
        :param manager_name: name of the manager the evaluator is from
        :type manager_name: str
        :param client_id: ID of the client at this manager
        :type client_id: int
        """

        self.__manager_client_id = client_id
        self.__cost_function_evaluator = cost_function_evaluator
        self.__cost_computable = True
        self.__registered_manager = manager_name

    def remove_cost_function_evaluator(self):
        """
        Deletes currently used cost_function_evaluator and makes it impossible
        to compute cost of tasks
        """

        self.__cost_computable = False
        del self.__cost_function_evaluator
        self.__cost_function_evaluator = None

    def check_possible_tasks(self):
        """
        Checks whether additional tasks are possible right now or not

        :return: whether additional tasks are possible right now
        """

        if len(self.__tasks) >= self.__max_tasks != -1:
            return False
        else:
            return True

    def get_delegation(self, auction_id):
        """
        Gets the delegation with this auction_id or raises an exception if
        there is no delegation with this id

        :param auction_id: auction_id of an existing delegation
        :return: the relevant delegation
        :raises LookupError: if there is no delegation with this auction_id
        """

        for delegation in self.__delegations:
            if delegation.get_auction_id() == auction_id:
                return delegation

        # if no delegation with this name exists
        raise LookupError("No delegation with the auction_id " + str(auction_id))

    def get_task_by_goal_name(self, goal_name):
        """
        Returns a currently running task with the given goal

        :param goal_name: name of the goal that the task has
        :return: the running task
        :raises LookupError: if there is no task with that goal
        """

        for task in self.__tasks:
            if task.goal_name() == goal_name:
                return task

        raise LookupError("No task with the goal named " + str(goal_name))

    def get_new_auction_id(self):
        """
        Creates a new auction_id for this instance of the DelegationManager

        :return: The new auction_id
        """

        self.__auction_id += 1

        # check that the auction_id is usable in ROS msgs/services
        uint32_max = 2**32 - 1
        if self.__auction_id > uint32_max:
            self.__logwarn("Space for auction IDs is exhausted, starting with 0 again. This could possibly lead to problems!")
            self.__auction_id = 0

        return self.__auction_id

    def get_name(self):
        """
        Gets the manager name

        :return: name of this delegation manager
        """

        return self._name

    def get_current_employers(self):
        employers = list()
        for t in self.__tasks:
            employers.extend(t.employers)

        return list(set(employers))

    # ------ Make auctions etc ------

    def __precom(self, proposal, delegation):
        """
        Sends a precom call for the delegation with this proposal

        Wrapper for the call

        :param proposal: the proposal that won
        :param delegation: the delegation with this proposal
        :return: response of the call
        """

        if self.depth_checking_possible:
            depth = self.__current_delegation_depth + 1
        else:
            depth = delegation.depth + 1

        employers = self.get_current_employers()
        employers.append(self._name)
        delegation_members = list(set(employers))

        response = self.__send_precom(target_name=proposal.get_name(),
                                      auction_id=delegation.get_auction_id(),
                                      goal_name=delegation.get_goal_name(),
                                      proposal_value=proposal.get_value(),
                                      goal_representation=delegation.get_goal_representation(),
                                      delegation_members=delegation_members,
                                      depth=depth)
        return response

    def __start_auction(self, delegation):
        """
        Starts the auction for this delegation with a CFP message

        :param delegation: the delegation for which the auction should be
                started
        """

        # Making sure that the delegation is in the right state
        delegation.start_auction()

        if self.depth_checking_possible:
            depth = self.__current_delegation_depth + 1
        else:
            depth = delegation.depth + 1

        auction_id = delegation.get_auction_id()
        goal_representation = delegation.get_goal_representation()

        employers = self.get_current_employers()
        employers.append(self._name)
        delegation_members = list(set(employers))

        self.__loginfo("Starting auction with ID: " + str(auction_id))
        try:
            self.__send_cfp(goal_representation=goal_representation, auction_id=auction_id, depth=depth, delegation_members=delegation_members)
        except DelegationServiceError as e:
            self.__logwarn("CFP was not sent right (error_message:\"" + str(e.message) + "\")")
            # restart auction next step by ending it than (it will restart,
            # because it has no valid proposals than)
            delegation.end_auction_next_step()

    def __end_auction(self, delegation):    # TODO myb clean this up a bit
        """
        Stops the auction for the given delegation, determines its winner and
        tries to make him the contractor

        :param delegation: the delegation of which the auction should end
        :return: whether it was successfully ended or not (restarted)
        :rtype: bool
        """

        auction_id = delegation.get_auction_id()
        self.__loginfo("Trying to end auction with ID " + str(auction_id))

        up_for_delegation = True

        for counter in range(self.MAX_CONSECUTIVE_TRIES):

            if not delegation.has_proposals():
                self.__logwarn("Auction with ID " + str(auction_id) + " has no proposals")
                break

            try:
                best_proposal = delegation.get_best_proposal()
            except LookupError:
                self.__logwarn("Auction with ID " + str(auction_id) + " has no proposals")
                break

            bidder_name = best_proposal.get_name()
            proposed_value = best_proposal.get_value()

            if bidder_name == self._name:
                self.__loginfo("I won my own auction with the ID " + str(auction_id))
                # make sure i do the work myself
                client_id = delegation.client_id
                client = DelegationClientBase.get_client(client_id=client_id)
                client.start_work_for_delegation(delegation_id=delegation.get_auction_id())
                up_for_delegation = False
                break

            self.__loginfo("Sending a precommit to " + str(bidder_name) + " who bid " + str(
                proposed_value) + " for my auction " + str(
                auction_id))

            try:
                response = self.__precom(proposal=best_proposal, delegation=delegation)
            except DelegationServiceError as e:
                # if the best bid is not reachable, try next best bid, instead of giving up auction
                self.__logwarn("Precommit failed, trying next best Bidder if applicable (error_message:\"" + str(e.message) + "\")")
                delegation.remove_proposal(proposal=best_proposal)
                continue

            manager_name = response.manager_name

            if response.acceptance:
                self.__loginfo(str(bidder_name) + " has accepted the contract for a cost of " + str(
                    proposed_value) + " for my auction " + str(
                    auction_id))

                try:
                    # set contractor, change delegation state and send goal
                    delegation.make_contract(bidder_name=bidder_name, manager_name=manager_name)
                except DelegationContractorError:
                    self.__logwarn("Contractor has already been chosen, while i am trying to find one")
                    up_for_delegation = False
                    break
                except DelegationError as e:
                    self.__logwarn("Sending goal was not possible! (error_message:\"" + str(e.message) + "\")")

                    # make sure goal is not living anymore and contractor is removed
                    delegation.terminate_contract()

                    delegation.remove_proposal(proposal=best_proposal)
                    continue

                # Contractor has been found
                up_for_delegation = False
                break

            elif response.still_biding:
                self.__loginfo(str(bidder_name) + " has given a new proposal of " + str(response.new_proposal)
                               + " for my auction " + str(auction_id))
                delegation.remove_proposal(proposal=best_proposal)

                if best_proposal.get_value() >= response.new_proposal:
                    self.__logwarn("The new proposal is not worse than the old proposal while he is not accepting the old proposal,"
                                   + " something is off!\nWont add this new proposal just to be safe")
                    continue

                try:
                    delegation.add_proposal(proposal=Proposal(bidder_name, response.new_proposal))
                except Warning:
                    self.__loginfo("Proposal from " + bidder_name + " wont be added, he is forbidden")

            else:
                self.__loginfo(str(bidder_name) + " has stopped biding for my auction " + str(auction_id))
                delegation.remove_proposal(proposal=best_proposal)

        if up_for_delegation:
            self.__logwarn("No possible contractor has been found for my auction " + str(auction_id))
            # starting a new delegation (real failure is currently not possible
            self.__start_auction(delegation=delegation)
            return False
        else:
            self.__loginfo("Auction with ID " + str(delegation.get_auction_id()) + " is finished")
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
            self.__loginfo("Trying to terminate delegation with the auction id " + str(auction_id) + ", that doesnt exist")
            return

        delegation.finish_delegation()

    def fail_task(self, goal_name):
        """
        Sends a message to the associated employer, that this task will no
        longer be pursued, if this is a task that comes from a different source

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
            self.__send_failure(auctioneer_name=task.get_auctioneer_name(), auction_id=task.get_auction_id())
        except DelegationServiceError as e:
            self.__logwarn("Failure Message raised following error: "+e.message+"\nWill try again")
            # TODO we would have to retry
            pass

    def delegate(self, goal_wrapper, client_id, auction_steps=None, own_cost=-1, known_depth=None):
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
            depth = self.__current_delegation_depth
        else:
            depth = -1

        if not self._check_depth(depth=depth):
            raise DelegationError("MAX_DELEGATION_DEPTH is reached")

        new = Delegation(goal_wrapper=goal_wrapper, auction_id=self.get_new_auction_id(),
                         auction_steps=auction_steps, client_id=client_id, depth=depth,
                         max_timeout_steps=self.MAX_CONSECUTIVE_TIMEOUTS)

        self.__delegations.append(new)
        self.__start_auction(delegation=new)

        if own_cost > 0:
            proposal = Proposal(name=self._name, value=own_cost)
            new.add_proposal(proposal=proposal)

        return new.get_auction_id()

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

        self.__logwarn("Maximum Delegation Depth is reached")
        return False

    def do_step(self, delegation_ids):
        """
        Performs a step for all delegations with a given Id

        :param delegation_ids: list of IDs of delegations that should be stepped
        :type delegation_ids: list
        """

        if len(delegation_ids) == 0:
            return

        self.__loginfo("Doing a step for auctions: "+" ".join(delegation_ids.__repr__()))

        delegations = [self.get_delegation(auction_id=i) for i in delegation_ids]
        waiting_delegations = [d for d in delegations if d.state.is_waiting_for_proposals()]
        running_delegations = [d for d in delegations if d.state.is_delegated_running()]

        for delegation in running_delegations:
            if not delegation.check_if_alive():
                self.__logwarn("Too many TIMEOUTS for the contractor \""+str(delegation.get_contractor())
                               + "\" of the auction with the ID "+str(delegation.get_auction_id())
                               + "! Will try to find a new contractor.")
                contractor_name = delegation.get_contractor()
                delegation.forbid_bidder(name=contractor_name)
                delegation.fail_current_delegation()  # unregisters goal
                self.__start_auction(delegation)

            if delegation.check_if_goal_finished():
                self.__loginfo("Delegation with ID "+str(delegation.get_auction_id())+" was successful")
                delegation.finish_delegation()
                client = DelegationClientBase.get_client(client_id=delegation.client_id)
                client.delegation_successful(delegation_id=delegation.get_auction_id())

        for delegation in waiting_delegations:
            # decrementing the needed steps and checking at the same time
            if delegation.decrement_and_check_steps():
                self.__end_auction(delegation=delegation)

    def end_task(self, goal_name):
        """
        Tries to end the task with a goal with this name

        :param goal_name: name of the goal that is no longer a task
        :type goal_name: str
        """

        try:
            task = self.get_task_by_goal_name(goal_name=goal_name)
            self.__tasks.remove(task)
            self.update_delegation_depth()
            self.__loginfo("Task with goal_name " + goal_name + " was finished")
            del task
        except LookupError:
            # this goal was no task given by a different manager
            return

    @property
    def depth_checking_possible(self):

        # is only given if cost computable
        return self.__cost_computable

    @property
    def cost_computable(self):
        return self.__cost_computable

#! /usr/bin/env python2

import rospy

from task_decomposition_module.msg import CFP
from task_decomposition_module.srv import Precommit, PrecommitResponse, \
    Propose, ProposeResponse, Failure, FailureResponse
from delegation_errors import DelegationServiceError, DelegationPlanningWarning, DelegationContractorError, DelegationError
from delegation import Delegation, Proposal
from task import Task
from goalwrapper import GoalWrapperBase


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
    cfp_topic_name = "CFP_Topic"
    SERVICE_TIMEOUT = 2     # should be configured according to system specs

    # ------ Initiation methods ------

    def __init__(self, instance_name="", max_tasks=0):
        """
        Constructor for the DelegationManager

        :param instance_name: name of this instance of the DelegationManager,
                should be unique
        :type instance_name: str
        :param max_tasks: number of maximum tasks that can simultaneously run,
                set this to 0 for no possible tasks
        :type max_tasks: int
        """

        self._name = instance_name

        self.__delegations = []
        self.__auction_id = 0

        self.__max_tasks = max_tasks
        self.__tasks = []

        self.__cost_function_evaluator = None
        self.__cost_computable = False
        self.__registered_manager = ""

        self.__init_topics()
        self.__init_services()

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

    # ------ Deletion methods ------

    def __del__(self):
        """
        Destructor for the DelegationManager
        """

        self.__logwarn("Stopping services and topics")
        self.__stop_services()
        self.__stop_topics()

        del self.__delegations[:]

    def __stop_services(self):
        """
        Shuts all services down
        """

        self._precom_service.shutdown()
        self._propose_service.shutdown()
        self._failure_service.shutdown()

    def __stop_topics(self):
        """
        Unregisters publisher and subscriber of this node
        """

        self._cfp_publisher.unregister()
        self._cfp_subscriber.unregister()

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
            return

        if not self.__cost_computable:
            # no cost computation available
            self.__loginfo("Wont bid, because i cannot compute the cost")
            return

        try:
            new_cost, possible_goal = self.__cost_function_evaluator.compute_cost_and_possibility(goal_representation=goal_representation)
        except DelegationPlanningWarning as e:
            self.__loginfo("Goal not possible. PlannerMessage: " + str(e.message))
            new_cost, possible_goal = -1, False

        if not possible_goal:
            self.__loginfo("Earlier proposal can not be verified, goal currently not possible")
            return response

        if new_cost <= request.old_proposal:
            self.__loginfo("Have accepted a contract from " + str(auctioneer_name))
            response.acceptance = True
            new_task = Task(auction_id=auction_id, auctioneer_name=auctioneer_name, goal_name=goal_name)
            self.add_task(new_task)
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

    def __failure_callback(self, request):      # TODO has to change (call unregister of the goal and update information)
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
        delegation.fail_current_delegation()
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

        self.__loginfo("Got CFP from " + str(auctioneer_name) + " with ID " + str(auction_id))

        # TODO should i bid this way for my OWN auctions? No!

        if not self.check_possible_tasks():
            # not bidding if i cannot take tasks right now or in general
            self.__loginfo("Wont bid, because i already have enough tasks or cannot take any")
            return

        if not self.__cost_computable:
            # no cost computation available
            self.__loginfo("Wont bid, because i cannot compute the cost")
            return

        try:
            cost, possible_goal = self.__cost_function_evaluator.compute_cost_and_possibility(goal_representation=goal_representation)
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

    def __send_precom(self, target_name, auction_id, proposal_value, goal_representation, goal_name):
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
            response = send_precom(goal_representation, self._name, goal_name, auction_id, proposal_value)
        except rospy.ServiceException:
            self.__logwarn("Precommit call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

        return response

    def __send_failure(self, auctioneer_name, auction_id):
        """
        Sends a Failure service-call for the specified name, id

        :param auctioneer_name: name of the auctioneer of the failed task
        :param auction_id: ID of the failed task
        :raises ServiceException: if call failed
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

    def __send_cfp(self, goal_representation, auction_id):
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

        try:
            self._cfp_publisher.publish(msg)
        except rospy.ROSException:
            self.__logwarn("CFP publish failed")
            raise DelegationServiceError("Call failed: CFP")

    # ------ Simple Getter/Setter for members ------

    def add_task(self, new_task):
        """
        Adds the task to the task list if possible

        :param new_task: the new task
        :raises Exception: if it cant be added
        """

        if len(self.__tasks) < self.__max_tasks:
            self.__tasks.append(new_task)
        else:
            raise Exception     # TODO specific exception

    def set_cost_function_evaluator(self, cost_function_evaluator, manager_name):
        """
        Adds a cost_function_evaluator, overwrites old evaluator if there is
        one and makes it possible for the delegation_manager to compute the cost
        of tasks with this evaluator

        :param cost_function_evaluator: a working cost_function_evaluator
        :type cost_function_evaluator: AbstractCostEvaluator
        :param manager_name: name of the manager the evaluator is from
        :type manager_name: str
        """

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

        if len(self.__tasks) >= self.__max_tasks:
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
            if task.get_goal_name() == goal_name:
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

    # ------ Make auctions etc ------

    def __precom(self, proposal, delegation):
        """
        Sends a precom call for the delegation with this proposal

        Wrapper for the call

        :param proposal: the proposal that won
        :param delegation: the delegation with this proposal
        :return: response of the call
        """

        response = self.__send_precom(target_name=proposal.get_name(),
                                      auction_id=delegation.get_auction_id(),
                                      goal_name=delegation.get_goal_name(),
                                      proposal_value=proposal.get_value(),
                                      goal_representation=delegation.get_goal_representation())
        return response

    def __start_auction(self, delegation):
        """
        Starts the auction for this delegation with a CFP message

        :param delegation: the delegation for which the auction should be
                started
        """

        # Making sure that the delegation is in the right state
        delegation.reset_proposals()
        delegation.reset_steps()
        delegation.state.set_waiting_for_proposal()

        auction_id = delegation.get_auction_id()
        goal_representation = delegation.get_goal_representation()

        self.__loginfo("Starting auction with ID: " + str(auction_id))
        try:
            self.__send_cfp(goal_representation=goal_representation, auction_id=auction_id)
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
        self.__loginfo("Trying to end Auction with ID " + str(auction_id))

        up_for_delegation = True

        while delegation.has_proposals() and up_for_delegation:

            try:
                best_proposal = delegation.get_best_proposal()
            except LookupError:
                self.__logwarn("Delegation with auction id " + str(auction_id) + " has no proposals")
                break

            bidder_name = best_proposal.get_name()
            proposed_value = best_proposal.get_value()

            self.__loginfo("Sending a precommit to " + str(bidder_name) + " who bid " + str(
                proposed_value) + " for my auction " + str(
                auction_id))

            try:
                response = self.__precom(proposal=best_proposal, delegation=delegation)
                manager_name = response.manager_name
            except DelegationServiceError as e:
                # if the best bid is not reachable, try next best bid, instead of giving up auction
                self.__logwarn("Precommit failed, trying next best Bidder if applicable (error_message:\"" + str(e.message) + "\")")
                delegation.remove_proposal(proposal=best_proposal)
                continue

            if response.acceptance:
                self.__loginfo(str(bidder_name) + " has accepted the contract for a cost of " + str(
                    proposed_value) + " for my auction " + str(
                    auction_id))

                try:
                    # set contractor and change delegation state
                    delegation.set_contractor(name=bidder_name)
                except DelegationContractorError:
                    self.__logwarn("Contractor has already been chosen, while i am trying to find one")
                    up_for_delegation = False
                    break

                # actually send the goal to the bidders Manager
                try:
                    rospy.loginfo("Trying to send the goal to the manager with the name " + str(manager_name))
                    delegation.send_goal(name=manager_name)
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
                self.__loginfo(str(bidder_name) + " has given a new proposal of " + str(response.new_proposal) + " for my auction " + str(
                    auction_id))
                delegation.remove_proposal(proposal=best_proposal)
                try:
                    delegation.add_proposal(proposal=Proposal(bidder_name, response.new_proposal))
                except Warning:
                    self.__loginfo("Proposal from " + bidder_name + " wont be added, he is forbidden")

            else:
                self.__loginfo(str(bidder_name) + " has stopped biding for my auction " + str(auction_id))
                delegation.remove_proposal(proposal=best_proposal)

        if up_for_delegation:
            self.__logwarn("No possible contractor has been found for my auction " + str(auction_id))
            # starting a new delegation (no failure in RHBP possible)
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
        Sends a message to the associated contractor, that this task will no
        longer be pursued, if this is a task that comes from a different source

        :param goal_name: name of the goal that failed
        :type goal_name: str
        """

        try:
            task = self.get_task_by_goal_name(goal_name=goal_name)
        except LookupError:
            # this goal was not given by a different manager
            return

        # send a failure right, if needed
        try:
            self.__send_failure(auctioneer_name=task.get_auctioneer_name(), auction_id=task.get_auction_id())
        except DelegationServiceError as e:
            # TODO
            pass

    def delegate(self, goal_wrapper, auction_steps=3):
        """
        Makes a delegation for the goal and starts an auction for this
        delegation

        The auction will be closed after the given number of steps were taken,
        a winner will be determined and the goal will be delegated to that
        winner.

        :param auction_steps: number of steps
                that are waited for proposals while the auction is running
        :type auction_steps: int
        :param goal_wrapper: wrapper for the goal that should be delegated
        :type goal_wrapper: GoalWrapperBase
        :return: the auction_id of the auction
        :rtype: int
        """

        new = Delegation(goal_wrapper=goal_wrapper, auction_id=self.get_new_auction_id(), auction_steps=auction_steps)

        self.__delegations.append(new)
        self.__start_auction(delegation=new)

        return new.get_auction_id()

    def do_step(self):
        """
        Does a step, meaning all delegations that are currently waiting
        for proposals are checked if their auction should end and those
        auctions are terminated
        """

        self.__loginfo("Doing a step")

        waiting_delegations = [d for d in self.__delegations if d.state.is_waiting_for_proposals()]

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
            self.__loginfo("Task with goal_name " + goal_name + " was finished")
            del task
        except LookupError:
            # this goal was no task given by a different manager
            return


class DelegationManagerSingleton(object):
    """
    Class that is a singleton container for the DelegationManager

    Initiate this class and use get_instance() and use that instance as a normal
    instance of the DelegationManager
    """

    __instance = None

    def __init__(self, manager_name="", max_tasks=0):
        """
        Constructor of the DelegationManagerSingleton

        Constructs a new instance of the DelegationManager if no instance has
        been created in this process

        This should be used first in the manager of process with the managers
        name, if a manager is used in this process/node

        :param manager_name: name of instance the DelegationManager, should be
                unique and the name of the normal manager if this
                DelegationManager can take tasks
        """

        if DelegationManagerSingleton.__instance is None:
            DelegationManagerSingleton.__instance = DelegationManager(instance_name=manager_name, max_tasks=max_tasks)
        else:
            # TODO myb change DelegationManager if formerly taking_tasks was false and this one is true
            rospy.loginfo("There is already an instance of the DelegationManager in this process")

    def get_instance(self):
        """
        Returns the instance of the DelegationManager if apparent

        :return: the instance of the DelegationManager class
        :raises RuntimeError: if no instance has been found
        """

        if DelegationManagerSingleton.__instance is None:
            raise RuntimeError("No instance has been found")
        return DelegationManagerSingleton.__instance


if __name__ == '__main__':

    # TODO this is just for testing purposes right now a passive manager

    name = "Default"

    rospy.init_node(name+"DelegationNode")
    dm = DelegationManager(name)

    rospy.loginfo("Starting Node with name \"" + name + "\"")

    rospy.loginfo("Spinning")
    rospy.spin()

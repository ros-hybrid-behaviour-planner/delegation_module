#! /usr/bin/env python2

import rospy

from task_decomposition_module.msg import CFP
from task_decomposition_module.srv import Precommit, PrecommitResponse, \
    Propose, ProposeResponse, Failure, FailureResponse, Terminate, \
    TerminateResponse
from delegation_errors import DelegationServiceError, DelegationPlanningWarning
from delegation import Delegation, Proposal
from task import Task


class DelegationManager(object):
    """
    This class represents the manager for all delegations of this agent.
    It only communicates with other instances of the DelegationManager,
    that represent other agents, and the general manager of its own agent.
    It handles possible delegations from taking the goal, that could be
    delegated, to making sure a delegated task is accomplished.

    The class contains the suffixes for services and the name of the CFP-topic.
    These have to be the same for all instances of the class.
    """

    precom_suffix = "/precom"
    propose_suffix = "/propose"
    failure_suffix = "/failure"
    terminate_suffix = "/terminate"
    cfp_topic_name = "CFP_Topic"

    # ------ Initiation methods ------

    def __init__(self, manager_name="", taking_tasks_possible=False, cost_function_evaluator=None):
        """
        Constructor for the DelegationManager

        :param manager_name: name of this instance of the DelegationManager,
                should be unique
        :param taking_tasks_possible: whether this is instance can actually
                take tasks or just delegate them
        :param cost_function_evaluator: an instance of a CostEvaluator
        """

        self._name = manager_name

        self.__delegations = []
        self.__auction_id = 0

        self.__taking_tasks_possible = taking_tasks_possible
        if taking_tasks_possible and cost_function_evaluator is None:
            rospy.logerr("Initiating a DelegationManager, that has to be able to take tasks without a cost-function!")
        self.__cost_function_evaluator = cost_function_evaluator
        self._got_task = False      # TODO myb more than one at the same time
        self.__running_task = None

        self.__init_topics()
        self.__init_services()

        self.__loginfo("Initiated DelelgationManager")

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
        self._terminate_service = rospy.Service(name=self._name+self.terminate_suffix, service_class=Terminate, handler=self.__terminate_callback)

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
        self._terminate_service.shutdown()

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
        """

        rospy.loginfo(str(self._name) + ": " + str(string))

    def __logwarn(self, string):
        """
        Rospy warn logging with information about who is logging

        :param string: Message to log
        """

        rospy.logwarn(str(self._name) + ": " + str(string))

    # ------ Callback functions ------

    def __terminate_callback(self, request):
        """
        Callback for terminate service calls

        Terminates currently running task

        :param request: request of the Terminate.srv type
        :return: empty response
        """

        auctioneer_name = request.name
        auction_id = request.auction_id
        self.__loginfo(str(auctioneer_name) + " is trying to terminate the task with his auction id " + str(auction_id))

        response = TerminateResponse()

        # check if the terminated task is really running
        if not self._got_task:
            self.__logwarn("Termination for a task, that i dont have")
            return response
        if self.__running_task.get_auction_id() != auction_id or self.__running_task.get_auctioneer_name() != auctioneer_name:
            self.__logwarn("Termination for a task, that i dont have")
            return response

        self.__loginfo("Stopping currently running task")

        # TODO kill corresponding goal from manager ( how to do this? )

        self._got_task = False
        self.__running_task = None

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

        self.__loginfo(str(auctioneer_name) + " sent a precommit for his auction " + str(auction_id))

        # response with initial values, all negative
        response = PrecommitResponse()
        response.acceptance = False
        response.still_biding = False
        response.new_proposal = 0

        if self._got_task or not self.__taking_tasks_possible:
            self.__loginfo("Taking a task is currently not possible")
            return response

        try:
            new_cost, possible_goal = self.__cost_function_evaluator(goal_representation)
        except DelegationPlanningWarning as e:
            self.__loginfo("Goal not possible. PlannerMessage: " + str(e.message))
            new_cost, possible_goal = -1, False

        if not possible_goal:
            self.__loginfo("Earlier proposal can not be verified, goal currently not possible")
            return response

        if new_cost <= request.old_proposal:
            self.__loginfo("Have accepted a contract from " + str(auctioneer_name))
            response.acceptance = True
            new_task = Task(auction_id=auction_id, auctioneer_name=auctioneer_name)
            self._got_task = True
            self.__running_task = new_task

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
        except NameError:
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
        except NameError:
            self.__logwarn("Failure Message for nonexistent delegation")
            return response

        try:
            delegation.get_contractor()
        except NameError:
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

        # TODO should i bid this way for my OWN auctions?

        if not self.__taking_tasks_possible:
            # Not bidding if i cannot perform tasks in general
            return

        if self._got_task:
            # not bidding while running tasks for someone else
            self.__loginfo("Wont bid, because i already have a task")
            return

        try:
            cost, possible_goal = self.__cost_function_evaluator(goal_representation)
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
            rospy.wait_for_service(service_name)
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

    def __send_precom(self, target_name, auction_id, proposal_value, goal_representation):
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
            rospy.wait_for_service(service_name)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_precom = rospy.ServiceProxy(service_name, Precommit)
            response = send_precom(goal_representation, self._name, auction_id, proposal_value)
        except rospy.ServiceException:
            self.__logwarn("Precommit call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

        return response

    def __send_terminate(self, target_name, auction_id):
        """
        Calls the Terminate service of the specified target for the auction with
        this ID

        :param target_name: name of the target of the call
        :param auction_id: ID of the auction for this termination
        :raises DelegationServiceError: if call failed
        """

        self.__loginfo("Sending a terminate to " + str(target_name) + " for my auction " + str(auction_id))

        service_name = target_name + self.terminate_suffix
        try:
            rospy.wait_for_service(service_name)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(service_name))
            raise DelegationServiceError("Waiting to long: " + str(service_name))

        try:
            send_terminate = rospy.ServiceProxy(service_name, Terminate)
            send_terminate(auction_id, self._name)
        except rospy.ServiceException:
            self.__logwarn("Terminate call failed")
            raise DelegationServiceError("Call failed: " + str(service_name))

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
            rospy.wait_for_service(service_name)
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

    def get_delegation(self, auction_id):
        """
        Gets the delegation with this auction_id or raises an exception if
        there is no delegation with this id

        :param auction_id: auction_id of an existing delegation
        :return: the relevant delegation
        :raises NameError: if there is no delegation with this auction_id
        """

        for delegation in self.__delegations:
            if delegation.get_auction_id() == auction_id:
                return delegation

        # if no delegation with this name exists
        raise NameError("No delegation with the auction_id " + str(auction_id))

    def get_task(self):     # TODO possibly more than one running task
        """
        Returns the currently running task

        :return: the running task
        """

        if self._got_task:
            return self.__running_task
        else:
            raise NameError("Got no task currently")

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
            # TODO what to do now?

    def __end_auction(self, delegation):    # TODO myb clean this up a bit
        """
        Stops the auction for the given delegation, determines its winner and
        tries to make him the contractor

        :param delegation: the delegation of which the auction should end
        :return: TODO to be determined, if any
        """

        auction_id = delegation.get_auction_id()
        self.__loginfo("Trying to end Auction with ID " + str(auction_id))

        up_for_delegation = True

        while delegation.has_proposals() and up_for_delegation:

            try:
                best_proposal = delegation.get_best_proposal()
            except NameError:
                self.__logwarn("Delegation with auction id " + str(auction_id) + " has no proposals")
                break

            bidder_name = best_proposal.get_name()
            proposed_value = best_proposal.get_value()

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

            if response.acceptance:
                self.__loginfo(str(bidder_name) + " has accepted the contract for a cost of " + str(
                    proposed_value) + " for my auction " + str(
                    auction_id))

                try:
                    # set contractor and change delegation state
                    delegation.set_contractor(name=bidder_name)
                except NameError:
                    self.__logwarn("Contractor has already been chosen, while i am trying to find one")
                    up_for_delegation = False
                    break

                # actually send the goal to the bidder
                try:
                    delegation.send_goal(name=bidder_name)
                except Exception as e:  # TODO really to broad / specify this exception when its chosen
                    self.__logwarn("Sending goal was not possible!")
                    # TODO make sure this works right at the side of the bidder (myb send terminate)
                    delegation.remove_proposal(proposal=best_proposal)
                    continue

                # Contractor has been found
                up_for_delegation = False

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

            # TODO Right handling, possible: send new CFP, give up or myb depending on needed delegation or just possible delegation

    # ------ Functions to interact with the DelegationManager ------

    def terminate(self, delegation):
        """
        Calls the Terminate service of the current contractor of this delegation

        Changes the state of the delegation to finished

        Wrapper for the call

        :param delegation: the delegation that should be terminated
        """

        contractor = delegation.get_contractor()
        auction_id = delegation.get_auction_id()

        self.__loginfo("Terminating contract with " + str(contractor) + " in my auction " + str(auction_id))
        self.__send_terminate(target_name=contractor, auction_id=auction_id)    # TODO catch exception/make sure termination is done right

        delegation.state.set_finished()

    def failure(self):
        """
        Sends a Failure service-call for the current task

        Wrapper for the call
        """

        # TODO catch exception/make sure failure is done right
        self.__send_failure(auctioneer_name=self.__running_task.get_auctioneer_name(), auction_id=self.__running_task.get_auction_id())

        # TODO possibly handle at a higher level
        self._got_task = False
        self.__running_task = None

    def delegate(self, goal_wrapper, auction_steps=3):
        """
        Makes a delegation for the goal and starts an auction for this
        delegation

        :param auction_steps: number of steps
                that are waited for proposals while the auction is running
        :param goal_wrapper: wrapper for the goal that should be delegated
        :return: the auction_id of the auction
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


class DelegationManagerSingleton(object):

    __instance = None

    def __init__(self, manager_name="", taking_tasks_possible=False, cost_function_evaluator=None):
        if self.__instance is None:
            self.__instance = DelegationManager(manager_name=manager_name, taking_tasks_possible=taking_tasks_possible, cost_function_evaluator=cost_function_evaluator)
        else:
            # TODO myb change DelegationManager if formerly taking_tasks was false and this one is true
            pass

    def get_instance(self):
        if self.__instance is None:
            pass
            # TODO raise right exception
        return self.__instance


if __name__ == '__main__':

    # TODO this is just for testing purposes right now a passive manager

    name = "Default"

    rospy.init_node(name+"DelegationNode")
    dm = DelegationManager(name)

    rospy.loginfo("Starting Node with name \"" + name + "\"")

    rospy.loginfo("Spinning")
    rospy.spin()


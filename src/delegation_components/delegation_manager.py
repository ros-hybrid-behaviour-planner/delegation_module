#! /usr/bin/env python2

import rospy

from task_decomposition_module.msg import CFP
from task_decomposition_module.srv import Precommit, PrecommitResponse, \
    Propose, ProposeResponse, Failure, FailureResponse, Terminate, \
    TerminateResponse
from delegation import Delegation, Proposal
from task import Task


class DelegationManager(object):
    # TODO more a declaration of intent than an actual documentation
    """
    This class represents the manager for all delegations of this agent.
    It only communicates with other instances of the DelegationManager,
    that represent other agents, and the general manager of its own agent.
    It handles possible delegations from taking the goal, that could be
    delegated, to making sure a delegated task is accomplished.
    """
    """
    The class contains the suffixes for services and the name of the CFP-topic.
    These have to be the same for all instances of the class.
    """
    precom_suffix = "/precom"
    propose_suffix = "/propose"
    failure_suffix = "/failure"
    terminate_suffix = "/terminate"
    cfp_topic_name = "CFP_Topic"

    # ------ Initiation methods ------

    def __init__(self, manager_name=""):
        """
        Constructor for the DelegationManager

        :param manager_name: name of this instance of the DelegationManager,
                should be unique
        """

        self._name = manager_name

        self.__delegations = []
        self.__auction_id = 0

        self._got_task = False      # TODO myb more than one at the same time
        self.__running_task = None

        self.__init_topics()
        self.__init_services()

        self.__loginfo("Initiated DelelgationManager")

    def __init_topics(self):
        """
        Initiates needed subscribers and publishers
        """

        self._cfp_publisher = rospy.Publisher(self.cfp_topic_name, CFP, queue_size=10)
        self._cfp_subscriber = rospy.Subscriber(self.cfp_topic_name, CFP, self.__cfp_callback)

    def __init_services(self):
        """
        Initiates needed services
        """

        self._precom_service = rospy.Service(self._name+self.precom_suffix, Precommit, self.__precom_callback)
        self._propose_service = rospy.Service(self._name+self.propose_suffix, Propose, self.__propose_callback)
        self._failure_service = rospy.Service(self._name+self.failure_suffix, Failure, self.__failure_callback)
        self._terminate_service = rospy.Service(self._name+self.terminate_suffix, Terminate, self.__terminate_callback)

    # ------ Deletion methods ------

    def __del__(self):
        """
        Destructor for the DelegationManager
        """

        self.__logwarn("Stopping services and topics")
        self.__stop_services()
        self.__stop_topics()

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

        self.__loginfo(str(request.name) + " is trying to terminate the task with his auction id " + str(request.auction_id))

        response = TerminateResponse()

        # check if the terminated task is really running
        if not self._got_task:
            return response
        if self.__running_task.get_auction_id() != request.auction_id \
                or self.__running_task.get_auctioneer_name() != request.name:
            return response

        self.__loginfo("Stopping currently running task")

        # TODO kill corresponding goal from manager

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

        self.__loginfo(str(request.name) + " sent a precommit for his auction " + str(request.auction_id))

        response = PrecommitResponse()

        if self._got_task:
            response.acceptance = False
            response.still_biding = False
            response.new_proposal = 0
            return response

        # TODO recheck cost and possibility
        new_cost = 5    # placeholder

        if new_cost <= request.old_proposal:

            self.__loginfo("Have accepted a contract from " + str(request.name))
            response.acceptance = True
            new_task = Task(request.auction_id, request.name)
            self._got_task = True
            self.__running_task = new_task

        else:
            self.__loginfo("Earlier proposed cost is lower than new cost:" + str(request.old_proposal) + "<" + str(new_cost))
            response.acceptance = False

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

        self.__loginfo(str(request.name) + " proposed for my auction " + str(request.auction_id) + " with " + str(request.value))

        try:
            delegation = self.get_delegation(request.auction_id)
        except NameError:
            self.__logwarn("Proposal for not existing delegation")
            return ProposeResponse()

        new_proposal = Proposal(request.name, request.value)
        try:
            delegation.add_proposal(new_proposal)
        except Warning:
            self.__loginfo("Proposal from " + new_proposal.get_name() + " wont be added, he is forbidden")

        return ProposeResponse()

    def __failure_callback(self, request):
        """
        Callback for failure service call
        Tries to make a new auction, because the old contractor failed to
        accomplish the task

        :param request: request of the Failure.srv type
        :return: empty response
        """

        self.__loginfo(str(request.name) + " reported a FAILURE for my auction " + str(request.auction_id))

        response = FailureResponse()
        try:
            delegation = self.get_delegation(request.auction_id)
        except NameError:
            self.__logwarn("Failure Message for not existing delegation")
            return response

        if delegation.get_contractor() != request.name:
            self.__logwarn("Failure Message from source who is not its contractor")
            return response

        delegation.forbid_bidder(request.name)
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

        self.__loginfo("Got CFP from " + str(msg.name) + " with ID " + str(msg.auction_id))

        # TODO should i bid this way for my OWN auctions?

        if self._got_task:
            # not bidding while running tasks for someone else
            self.__loginfo("Wont bid, because i already have a task")
            return

        # TODO try planning, compute cost

        cost = 5    # placeholder
        possible_goal = True    # placeholder

        if possible_goal:
            self.__loginfo("Sending a proposal of " + str(cost))
            try:
                self.__send_propose(cost, msg.name, msg.auction_id)
            except rospy.ServiceException:
                # Sending of a Proposal is not ultimately important
                self.__logwarn("Sending of Proposal not working, continuing without")

    # ------ Message sending methods ------

    def __send_propose(self, value, target_name, auction_id):
        """
        Sends a Propose service call

        :param value: proposed cost
        :param target_name: name of the auctioneer
        :param auction_id: ID of the auction
        :raises ServiceException: if call failed
        """

        self.__loginfo("Sending a proposal to " + str(target_name) + " for his auction " + str(auction_id))

        try:
            rospy.wait_for_service(target_name + self.propose_suffix)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(target_name + self.propose_suffix))
            raise rospy.ServiceException()

        try:
            send_proposal = rospy.ServiceProxy(
                target_name + self.propose_suffix, Propose)
            send_proposal(self._name, auction_id, value)
        except rospy.ServiceException:
            self.__logwarn("Propose call failed")
            raise

    def __send_precom(self, target_name, auction_id, proposal_value, goal_represenation):
        """
        Calls the Precommit service of the winning bidder of this delegation

        :param target_name: name of the bidder who gets the precom
        :param auction_id: ID of the corresponding auction
        :param proposal_value: proposed value
        :param goal_represenation: represenation of the goal for this auction
        :return: response of the service call,
                includes the acceptance of the bidder or possibly a new proposal
        :raises ServiceException: if call failed
        """

        self.__loginfo("Sending a precommit to " + str(target_name) + " for my auction " + str(auction_id))

        try:
            rospy.wait_for_service(target_name + self.precom_suffix)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(target_name + self.precom_suffix))
            raise rospy.ServiceException()

        try:
            send_precom = rospy.ServiceProxy(target_name + self.precom_suffix, Precommit)
            response = send_precom(goal_represenation, self._name, auction_id, proposal_value)
        except rospy.ServiceException:
            self.__logwarn("Precommit call failed")
            raise

        return response

    def __send_terminate(self, target_name, auction_id):
        """
        Calls the Terminate service of the specified target for the auction with
        this ID

        :param target_name: name of the target of the call
        :param auction_id: ID of the auction for this termination
        :raises ServiceException: if call failed
        """

        self.__loginfo("Sending a terminate to " + str(target_name) + " for my auction " + str(auction_id))

        try:
            rospy.wait_for_service(target_name + self.terminate_suffix)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(target_name + self.terminate_suffix))
            raise rospy.ServiceException()

        try:
            send_terminate = rospy.ServiceProxy(target_name + self.terminate_suffix, Terminate)
            send_terminate(auction_id, self._name)
        except rospy.ServiceException:
            self.__logwarn("Terminate call failed")
            raise

    def __send_failure(self, auctioneer_name, auction_id):
        """
        Sends a Failure service-call for the specified name, id

        :param auctioneer_name: name of the auctioneer of the failed task
        :param auction_id: ID of the failed task
        :raises ServiceException: if call failed
        """

        self.__loginfo("Sending a Failure message to " + str(auctioneer_name) + " for his auction " + str(auction_id))

        try:
            rospy.wait_for_service(auctioneer_name + self.failure_suffix)
        except rospy.ROSException:
            self.__logwarn("Waiting to long for service: " + str(auctioneer_name + self.failure_suffix))
            raise rospy.ServiceException()

        try:
            send_failure = rospy.ServiceProxy(auctioneer_name + self.failure_suffix, Failure)
            send_failure(self._name, auction_id)
        except rospy.ServiceException:
            self.__logwarn("Failure call failed")
            raise

    def __send_cfp(self, goal_representation, auction_id):
        """
        Sends a CFP-broadcast over the CFP-topic

        :param goal_representation: goal_information
        :param auction_id: ID of the corresponding auction
        :raises ServiceException: if call failed
        """

        self.__loginfo("Sending a CFP for my auction " + str(auction_id))

        msg = CFP()
        msg.pddlstring = goal_representation
        msg.name = self._name
        msg.auction_id = auction_id

        try:
            self._cfp_publisher.publish(msg)
        except rospy.ROSException:
            self.__logwarn("CFP publish failed")
            raise rospy.ServiceException()

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

        uint32_max = 2**32 - 1
        if self.__auction_id > uint32_max:
            self.__logwarn("Space for auction IDs is exhausted, starting with 0 again")
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

        response = self.__send_precom(proposal.get_name(),
                                      delegation.get_auction_id(),
                                      proposal.get_value(),
                                      delegation.get_goal_representation())
        return response

    def __start_auction(self, delegation):
        """
        Starts the auction for this delegation with a CFP message

        :param delegation: the delegation for which the auction should be
                started
        """

        delegation.reset_proposals()

        self.__loginfo("Starting auction with ID: " + str(delegation.get_auction_id()))

        self.__send_cfp(delegation.get_goal_representation(), delegation.get_auction_id())

    def terminate(self, delegation):
        """
        Calls the Terminate service of the current contractor of this delegation
        Wrapper for the call

        :param delegation: the delegation that should be terminated
        """

        self.__loginfo("Terminating contract with " + str(delegation.get_contractor()) + " in my auction " + str(delegation.get_auction_id()))

        self.__send_terminate(delegation.get_contractor(), delegation.get_auction_id())

    def failure(self):
        """
        Sends a Failure service-call for the current task
        Wrapper for the call
        """

        self.__send_failure(self.__running_task.get_auctioneer_name(), self.__running_task.get_auction_id())

        # TODO possibly handle at a higher level
        self._got_task = False
        self.__running_task = None

    def delegate(self, goal):       # TODO subject to change, parameter?
        """
        Makes a delegation for the goal and starts an auction for this
        delegation

        :param goal: the goal that should be delegated   TODO stc
        :return: the auction_id of the auction
        """

        new = Delegation(goal, self.get_new_auction_id())

        self.__delegations.append(new)
        self.__start_auction(new)

        # TODO myb start a thread in which is waited for a the proposals...

        return new.get_auction_id()

    def end_auction(self, delegation):
        """
        Stops the auction for the given delegation, determines its winner and
        tries to make him the contractor

        :param delegation: the delegation of which the auction should end
        :return: TODO to be determined, if any
        """

        up_for_delegation = True

        while delegation.has_proposals() and up_for_delegation:

            best_proposal = delegation.get_best_proposal()
            self.__loginfo("Sending a precommit to " + str(best_proposal.get_name()) + " who bid " + str(best_proposal.get_value()) + " for my auction " + str(delegation.get_auction_id()))

            try:
                response = self.__precom(best_proposal, delegation)
            except rospy.ServiceException:
                # if the best bid is not reachable, try next best bid, instead of giving up auction
                self.__logwarn("Precommit failed, trying next best Bidder if applicable")
                delegation.remove_proposal(best_proposal)
                continue

            if response.acceptance:

                self.__loginfo(str(best_proposal.get_name()) + " has accepted the contract for a cost of " + str(best_proposal.get_value()) + " for my auction " + str(delegation.get_auction_id()))
                delegation.set_contractor(best_proposal.get_name())

                # TODO send goal to contractor

                # TODO set delegation status right

                # Contractor has been found
                up_for_delegation = False

            elif response.still_biding:

                self.__loginfo(str(best_proposal.get_name()) + " has given a new proposal of " + str(response.new_proposal) + " for my auction " + str(delegation.get_auction_id()))
                delegation.remove_proposal(best_proposal)
                try:
                    delegation.add_proposal(Proposal(best_proposal.get_name(), response.new_proposal))
                except Warning:
                    self.__loginfo("Proposal from " + best_proposal.get_name() + " wont be added, he is forbidden")

            else:

                self.__loginfo(str(best_proposal.get_name()) + " has stopped biding for my auction " + str(delegation.get_auction_id()))
                delegation.remove_proposal(best_proposal)

        if up_for_delegation:
            self.__logwarn("No possible contractor has been found for my auction " + str(delegation.get_auction_id()))
            # TODO Right handling, possible: send new CFP, give up or myb depending on needed delegation or just possible delegation


if __name__ == '__main__':

    # TODO this is just for testing purposes right now, just a passive manager

    name = "Default"

    rospy.init_node(name+"DelegationNode")
    dm = DelegationManager(name)

    rospy.loginfo("Starting Node with name \"" + name + "\"")

    rospy.loginfo("Spinning")
    rospy.spin()


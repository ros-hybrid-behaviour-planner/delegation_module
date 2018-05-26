#! /usr/bin/env python2

import sys
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
    cfp_topic_name = "CFP-topic"

    def __init__(self, manager_name=""):
        """
        Constructor for the DelegationManager

        :param manager_name: name of this instance of the DelegationManager,
                should be unique
        """
        self._name = manager_name
        self.__delegations = []
        self.__got_task = False
        self.__running_task = None
        self.__auction_id = 0

        self.__init_services()
        self.__init_topics()

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

    def __del__(self):
        """
        Destructor for the DelegationManager
        """
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

    def __terminate_callback(self, request):
        """
        Callback for terminate service calls
        Terminates currently running task

        :param request: request of the Terminate.srv type
        :return: empty response
        """

        rospy.loginfo(str(request.name) + " is trying to terminate the task with his auction id " + str(request.auction_id))

        response = TerminateResponse()

        # check if the terminated task is really running
        if not self.__got_task:
            return response
        if self.__running_task.get_auction_id() != request.auction_id \
                or self.__running_task.get_auctioneer_name() != request.name:
            return response

        rospy.loginfo("Stopping currently running task")

        # TODO kill corresponding goal from manager

        self.__got_task = False
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

        rospy.loginfo(str(request.name) + " sent a precommit for his auction " + str(request.auction_id))

        response = PrecommitResponse()

        if self.__got_task:
            response.acceptance = False
            response.still_biding = False
            response.new_proposal = 0
            return response

        # TODO recheck cost and possibility
        new_cost = 5    # placeholder

        if new_cost <= request.old_proposal:

            rospy.loginfo("Have accepted a contract from " + str(request.name))
            response.acceptance = True
            new_task = Task(request.auction_id, request.name)
            self.__got_task = True
            self.__running_task = new_task

        else:
            rospy.loginfo("Earlier proposed cost is lower than new cost:" + str(request.old_proposal) + "<" + str(new_cost))
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

        rospy.loginfo(str(request.name) + " proposed for my auction " + str(request.auction_id) + " with " + str(request.value))

        delegation = self.get_delegation(request.auction_id)
        new_proposal = Proposal(request.name, request.value)
        delegation.add_proposal(new_proposal)
        return ProposeResponse()

    def __failure_callback(self, request):
        """
        Callback for failure service call
        Tries to make a new auction, because the old contractor failed to
        accomplish the task

        :param request: request of the Failure.srv type
        :return: empty response
        """

        rospy.loginfo(str(request.name) + " reported a FAILURE for my auction " + str(request.auction_id))

        response = FailureResponse()
        try:
            delegation = self.get_delegation(request.auction_id)
        except Exception as e:  # TODO implement special exception and only catch that
            return response

        # TODO make sure old contractor doesnt get this contract again
        self._start_auction(delegation)

        return response

    def __cfp_callback(self, msg):
        """
        Callback for messages in the CFP topic, defined in self._topic_name
        Determines if a proposal for this CFP should be made and makes one
        if appropriate

        :param msg: message of the CFP.msg type
        """

        rospy.loginfo("Got CFP from " + str(msg.name) + " with ID " + str(msg.auction_id))

        if self.__got_task:
            # not bidding while running tasks for someone else
            return

        # TODO try planning, compute cost

        cost = 5    # placeholder
        possible_goal = True    # placeholder

        if possible_goal:
            rospy.wait_for_service(msg.name+self.propose_suffix)
            send_proposal = rospy.ServiceProxy(msg.name+self.propose_suffix, Propose)
            send_proposal(self._name, msg.auction_id, cost)
            # TODO catch exceptions
        # else nothing has to be done

    def __send_precom(self, delegation, best_proposal):
        """
        Calls the Precommit service of the winning bidder of this delegation

        :param delegation: delegation for which the winner should be called
        :param best_proposal: proposal of the winner, includes name of the
                winner
        :return: response of the service call,
                includes the acceptance of the bidder or possibly a new proposal
        """
        rospy.wait_for_service(best_proposal.get_name() + self.precom_suffix)
        send_precom = rospy.ServiceProxy(best_proposal.get_name() + self.precom_suffix, Precommit)
        response = send_precom(delegation.get_goal_representation(), self._name, delegation.get_auction_id(), best_proposal.get_value())
        return response

    def send_terminate(self, delegation):
        """
        Calls the Terminate service of the current contractor of this delegation

        :param delegation: the delegation that should be terminated
        """

        rospy.loginfo("Terminating contract with " + str(delegation.get_contractor()) + " in my auction " + str(delegation.get_auction_id()))

        rospy.wait_for_service(delegation.get_contractor() + self.terminate_suffix)
        send_terminate = rospy.ServiceProxy(delegation.get_contractor() + self.terminate_suffix, Terminate())
        send_terminate(delegation.get_auction_id(), self._name)

    def _start_auction(self, delegation):
        """
        Starts the auction for this delegation with a CFP message

        :param delegation: the delegation for which the auction should be
                started
        """

        delegation.reset_proposals()

        rospy.loginfo("Starting auction with ID: " + str(delegation.get_auction_id()))

        msg = CFP()
        msg.pddlstring = delegation.get_goal_representation()
        msg.name = self._name
        msg.auction_id = delegation.get_auction_id()

        # TODO catch exceptions myb
        self._cfp_publisher.publish(msg)

    def get_delegation(self, auction_id):
        """
        Gets the delegation with this auction_id or raises an exception if
        there is no delegation with this id

        :param auction_id: auction_id of an existing delegation
        :return: the relevant delegation
        """
        for delegation in self.__delegations:
            if delegation.get_auction_id() == auction_id:
                return delegation
        # TODO raise exception if no delegation with this id

    def new_auction_id(self):
        """
        Creates a new auction_id for this instance of the DelegationManager

        :return: The new auction_id
        """
        self.__auction_id += 1
        return self.__auction_id

    def delegate(self, goal):       # TODO subject to change, parameter?
        """
        Makes a delegation for the goal and starts an auction for this
        delegation

        :param goal: the goal that should be delegated   TODO stc
        :return: the auction_id of the auction
        """
        new = Delegation(goal, self.new_auction_id())

        self.__delegations.append(new)
        self._start_auction(new)

        return new.get_auction_id()

    def end_auction(self, delegation):
        """
        Stops the auction for the given delegation, determines its winner and
        tries to make him the contractor

        :param delegation: the delegation of which the auction should end
        :return: TODO to be determined
        """

        not_delegated = True

        while not_delegated:

            if not delegation.has_proposals():
                # TODO myb different handling if no proposals are there
                return False

            best_proposal = delegation.get_best_proposal()
            rospy.loginfo("Sending a precommit to " + str(best_proposal.get_name()) + " who bid " + str(best_proposal.get_value()) + " for my auction " + str(delegation.get_auction_id()))

            response = self.__send_precom(delegation, best_proposal)
            # TODO catch exceptions

            if response.acceptance:
                # TODO send goal to contractor

                rospy.loginfo(str(best_proposal.get_name()) + " has accepted the contract for a cost of " + str(best_proposal.get_value()) + " for my auction " + str(delegation.get_auction_id()))
                delegation.set_contractor(best_proposal.get_name())

                not_delegated = False

            elif response.still_biding:

                rospy.loginfo(str(best_proposal.get_name()) + " has given a new proposal of " + str(response.new_proposal) + " for my auction " + str(delegation.get_auction_id()))
                delegation.remove_proposal(best_proposal)
                delegation.add_proposal(Proposal(best_proposal.get_name, response.new_proposal))

            else:

                rospy.loginfo(str(best_proposal.get_name()) + " has stopped biding for my auction " + str(delegation.get_auction_id()))
                delegation.remove_proposal(best_proposal)


if __name__ == '__main__':

    # TODO this is just for testing purposes right now

    name = "Default"
    posting = "True"

    for arg in sys.argv:
        if arg.startswith('name:='):
            name = arg[len('name:='):]
        if arg.startswith('posting:='):
            posting = arg[len('posting:='):]

    rospy.init_node(name+"DelegationNode")
    dm = DelegationManager(name)

    rospy.loginfo("Starting Node with name \"" + name + "\"")

    if posting == "False":
        print "Not Posting"
        rospy.spin()

    while True:

        rospy.sleep(5)

        del_id = dm.delegate("test_delegation")

        rospy.sleep(5)

        dm.end_auction(dm.get_delegation(del_id))

        rospy.sleep(5)

        dm.send_terminate(dm.get_delegation(del_id))

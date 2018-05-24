#! /usr/bin/env python2


import rospy
from behaviour_components.goals import Goal
from task_decomposition_module.msg import CFP
from task_decomposition_module.srv import Precommit, PrecommitResponse, Propose, ProposeResponse, Failure, FailureResponse, Terminate, TerminateResponse
from delegation import Delegation, Proposal
from task import Task


class DelegationManager(object):

    precom_suffix = "/precom"
    propose_suffix = "/propose"
    failure_suffix = "/failure"
    terminate_suffix = "/terminate"

    def __init__(self, name="", topic_name="CFP_topic"):
        self._name = name
        self._topic_name = topic_name
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
        self._cfp_publisher = rospy.Publisher(self._topic_name, CFP)
        self._cfp_subscriber = rospy.Subscriber(self._topic_name, CFP, self.__cfp_callback)

    def __init_services(self):
        """
        Initiates needed services
        """
        self._precom_service = rospy.Service(self._name+self.precom_suffix, Precommit, self.__precom_callback)
        self._propose_service = rospy.Service(self._name+self.propose_suffix, Propose, self.__propose_callback)
        self._failure_service = rospy.Service(self._name+self.failure_suffix, Failure, self.__failure_callback)
        self._terminate_service = rospy.Service(self._name+self.terminate_suffix, Terminate, self.__terminate_callback)

    def __terminate_callback(self, request):
        response = TerminateResponse()

        # check if the terminated task is really running
        if not self.__got_task:
            return response
        if self.__running_task.get_auction_id() != request.auction_id \
                or self.__running_task.get_auctioneer_name() != request.name:
            return response

        # TODO kill corresponding goal from manager

        self.__got_task = False
        self.__running_task = None

        return response

    def __precom_callback(self, request):

        response = PrecommitResponse()

        if self.__got_task:
            response.still_biding = False
            return response

        # TODO recheck cost and possibility
        new_cost = 5    # placeholder

        if new_cost == request.old_proposal:
            response.acceptance = True
            new_task = Task(request.auction_id, request.name)
            self.__got_task = True
            self.__running_task = new_task

        else:
            response.acceptance = False

        response.still_biding = True
        response.new_proposal = new_cost
        return response

    def __propose_callback(self, request):
        delegation = self.get_delegation(request.auction_id)
        new_proposal = Proposal(request.name, request.value)
        delegation.add_proposal(new_proposal)
        return ProposeResponse()

    def __failure_callback(self, request):

        response = FailureResponse()
        try:
            delegation = self.get_delegation(request.auction_id)
        except Exception as e:  # TODO implement specil exception nd only catch that
            return response

        # TODO make sure old contractor doesnt get this contract again
        self._start_auction(delegation)

        return response

    def __cfp_callback(self, msg):

        possible_goal = False

        # TODO try planning, compute cost
        cost = 5    # placeholder

        if possible_goal:
            request = Propose()
            request.name = self._name
            request.value = cost
            request.auction_id = msg.auction_id
            rospy.wait_for_service(msg.name+self.propose_suffix)
            send_proposal = rospy.ServiceProxy(msg.name+self.propose_suffix, Proposal)
            send_proposal(request)
            # TODO catch exceptions
        # else nothing has to be done

    def _start_auction(self, delegation):

        delegation.reset_proposals()

        msg = CFP()
        msg.pddlstring = delegation.get_pddl()
        msg.name = self._name
        msg.auction_id = delegation.get_auction_id()

        # TODO catch exceptions myb
        self._cfp_publisher.publish(msg)

    def get_delegation(self, auction_id):
        for delegation in self.__delegations:
            if delegation.get_auction_id() == auction_id:
                return delegation
        # TODO raise exception if no delegation with this id

    def new_auction_id(self):
        self.__auction_id += 1
        return self.__auction_id

    def delegate(self, goal):
        new = Delegation(goal, self.new_auction_id())

        self.__delegations.append(new)
        self._start_auction(new)

    def end_auction(self, delegation):

        best_proposal = delegation.get_best_proposal()

        request = Precommit()
        request.pddlstring = delegation.get_pddl()
        request.name = self._name
        request.auction_id = delegation.get_auction_id()
        request.old_proposal = best_proposal.get_value()
        rospy.wait_for_service(best_proposal.get_name()+self.precom_suffix)
        send_precom = rospy.ServiceProxy(best_proposal.get_name()+self.precom_suffix, Precommit)
        response = send_precom(request)
        # TODO catch exceptions
        # TODO evaluate answer





#! /usr/bin/env python2

import rospy

from delegation_components.delegation_manager import DelegationManager
from task_decomposition_module.msg import CFP
from task_decomposition_module.srv import Precommit, PrecommitResponse, \
    Propose, ProposeResponse, Failure, FailureResponse
from rhbp_core.srv import AddGoal, AddGoalResponse
from delegation_components.cost_computing import AbstractCostEvaluator


class AlwaysWinningCostEvaluator(AbstractCostEvaluator):

    def __init__(self):
        super(AlwaysWinningCostEvaluator, self).__init__()

    def compute_cost_and_possibility(self, goal_representation):
        self._last_cost, self._last_possibility = 0, True
        return 0, True


class DelegationMockingNode(object):

    def __init__(self, name, manager_name):
        self._name = name
        self._manager_name = manager_name
        self._service_prefix = manager_name + '/'
        self._cfp_publisher = rospy.Publisher(name=DelegationManager.cfp_topic_name, data_class=CFP, queue_size=10)
        self._cfp_subscriber = rospy.Subscriber(name=DelegationManager.cfp_topic_name, data_class=CFP, callback=self.__cfp_callback)
        self._precom_service = rospy.Service(name=self._name + DelegationManager.precom_suffix, service_class=Precommit, handler=self.__precom_callback)
        self._propose_service = rospy.Service(name=self._name + DelegationManager.propose_suffix, service_class=Propose, handler=self.__propose_callback)
        self._failure_service = rospy.Service(name=self._name + DelegationManager.failure_suffix, service_class=Failure, handler=self.__failure_callback)
        self.__addGoalService = rospy.Service(self._service_prefix + 'AddGoal', AddGoal, self.__add_goal_callback)
        # Precom
        self.PAcceptance = False
        self.PBidding = False
        self.PNewCost = 0
        self.Pre_last = None
        # Propose
        self.Pro_last = None
        # Failure
        self.Fail_last = None
        # CFP
        self.CFP_last = None
        # AddGoal
        self.AddG_last = None

    def __precom_callback(self, request):

        self.Pre_last = request

        # response with initial values, all negative
        response = PrecommitResponse()
        response.acceptance = False
        response.still_biding = False
        response.new_proposal = 0
        response.manager_name = ""

        return response

    def __propose_callback(self, request):

        self.Pro_last = request

        return ProposeResponse()

    def __failure_callback(self, request):

        self.Fail_last = request

        return FailureResponse

    def __cfp_callback(self, msg):

        self.CFP_last = msg

    def __add_goal_callback(self, request):

        self.AddG_last = request

        return AddGoalResponse()

    #
    # Sending
    #

    def send_propose(self, value, target_name, auction_id):

        service_name = target_name + DelegationManager.propose_suffix
        rospy.wait_for_service(service=service_name, timeout=DelegationManager.SERVICE_TIMEOUT)

        send_proposal = rospy.ServiceProxy(
            service_name, Propose)
        response = send_proposal(self._name, auction_id, value)
        return response

    def send_precom(self, target_name, auction_id, proposal_value, goal_representation, goal_name):

        service_name = target_name + DelegationManager.precom_suffix
        rospy.wait_for_service(service=service_name, timeout=DelegationManager.SERVICE_TIMEOUT)

        send_precom = rospy.ServiceProxy(service_name, Precommit)
        response = send_precom(goal_representation, self._name, goal_name, auction_id, proposal_value)

        return response

    def send_failure(self, auctioneer_name, auction_id):

        service_name = auctioneer_name + DelegationManager.failure_suffix
        rospy.wait_for_service(service=service_name, timeout=DelegationManager.SERVICE_TIMEOUT)

        send_failure = rospy.ServiceProxy(service_name, Failure)
        response = send_failure(self._name, auction_id)
        return response

    def send_cfp(self, goal_representation, auction_id):

        msg = CFP()
        msg.goal_representation = goal_representation
        msg.name = self._name
        msg.auction_id = auction_id

        self._cfp_publisher.publish(msg)


if __name__ == '__main__':

    rospy.init_node(name="PassiveNode")

    rospy.loginfo("Starting a passive node with the basic scenario")

    passive_manager = DelegationMockingNode(name="passive_delegation", manager_name="passive_manager")

    rospy.spin()




#! /usr/bin/env python2

import rospy

from delegation_components.delegation_manager import DelegationManager
from task_decomposition_module.msg import CFP
from task_decomposition_module.srv import Precommit, PrecommitResponse, \
    Propose, ProposeResponse, Failure, FailureResponse
from rhbp_core.srv import AddGoal, AddGoalResponse
from delegation_components.cost_computing import AbstractCostEvaluator
from delegation_components.goalwrapper import GoalWrapperBase


class MockedCostEvaluator(AbstractCostEvaluator):

    def __init__(self, cost, possibility):
        super(MockedCostEvaluator, self).__init__()
        self._last_cost = cost
        self._last_possibility = possibility

    def compute_cost_and_possibility(self, goal_representation):

        return self._last_cost, self._last_possibility


class MockedGoalWrapper(GoalWrapperBase):

    def __init__(self, name):
        super(MockedGoalWrapper, self).__init__(name=name)
        self._manager_name = ""

    def send_goal(self, name):
        self._manager_name = name
        self._created_goal = True

    def get_manager(self):
        return self._manager_name

    def terminate_goal(self):
        self._created_goal = False

    def get_goal_representation(self):
        return self._name


class MockedDelegationCommunicator(object):

    def __init__(self, name, manager_name):
        self._name = name
        self._manager_name = manager_name
        self._service_prefix = manager_name + '/'
        self._cfp_publisher = rospy.Publisher(name=DelegationManager.cfp_topic_name, data_class=CFP, queue_size=10)
        self._cfp_subscriber = rospy.Subscriber(name=DelegationManager.cfp_topic_name, data_class=CFP, callback=self.__cfp_callback)
        self._precom_service = rospy.Service(name=self._name + DelegationManager.precom_suffix, service_class=Precommit, handler=self.__precom_callback)
        self._propose_service = rospy.Service(name=self._name + DelegationManager.propose_suffix, service_class=Propose, handler=self.__propose_callback)
        self._failure_service = rospy.Service(name=self._name + DelegationManager.failure_suffix, service_class=Failure, handler=self.__failure_callback)
        self._addGoalService = rospy.Service(self._service_prefix + 'AddGoal', AddGoal, self.__add_goal_callback)
        # Precom
        self.PAcceptance = False
        self.PBidding = False
        self.PNewCost = 0
        self.Pre_last = None
        self.got_pre = False
        # Propose
        self.Pro_last = None
        self.got_pro = False
        # Failure
        self.Fail_last = None
        self.got_fail = False
        # CFP
        self.CFP_last = None
        self.got_cfp = False
        # AddGoal
        self.AddG_last = None
        self.got_addg = False

    def start_communication(self):
        self._precom_service = rospy.Service(name=self._name + DelegationManager.precom_suffix, service_class=Precommit, handler=self.__precom_callback)
        self._propose_service = rospy.Service(name=self._name + DelegationManager.propose_suffix, service_class=Propose, handler=self.__propose_callback)
        self._failure_service = rospy.Service(name=self._name + DelegationManager.failure_suffix, service_class=Failure, handler=self.__failure_callback)
        self._addGoalService = rospy.Service(self._service_prefix + 'AddGoal', AddGoal, self.__add_goal_callback)

    def __del__(self):
        """
        Destructor for the DelegationManager
        """

        self.stop_communication()
        self._cfp_publisher.unregister()
        self._cfp_subscriber.unregister()

    def stop_communication(self):

        self._precom_service.shutdown()
        self._propose_service.shutdown()
        self._failure_service.shutdown()
        self._addGoalService.shutdown()

    def __precom_callback(self, request):

        print("Got Precom Request")

        self.Pre_last = request
        self.got_pre = True

        # response with initial values, all negative
        response = PrecommitResponse()
        response.acceptance = self.PAcceptance
        response.still_biding = self.PBidding
        response.new_proposal = self.PNewCost
        response.manager_name = self._manager_name

        return response

    def __propose_callback(self, request):

        print("Got Propose Request")

        self.Pro_last = request
        self.got_pro = True

        return ProposeResponse()

    def __failure_callback(self, request):

        print("Got Failure Request")

        self.Fail_last = request
        self.got_fail = True

        return FailureResponse()

    def __cfp_callback(self, msg):

        print("Got CFP")

        self.CFP_last = msg
        self.got_cfp = True

    def __add_goal_callback(self, request):

        print("Got AddGoal request")

        self.AddG_last = request
        self.got_addg = True

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

    def reset_messages(self):
        # Precom
        self.Pre_last = None
        self.got_pre = False
        # Propose
        self.Pro_last = None
        self.got_pro = False
        # Failure
        self.Fail_last = None
        self.got_fail = False
        # CFP
        self.CFP_last = None
        self.got_cfp = False
        # AddGoal
        self.AddG_last = None
        self.got_addg = False

    def set_precom_response(self, acceptance, still_bidding, cost):
        self.PAcceptance = acceptance
        self.PBidding = still_bidding
        self.PNewCost = cost


if __name__ == '__main__':

    rospy.init_node(name="PassiveNode")

    rospy.loginfo("Starting a completely passive node that can answer DelegationServiceRequests, but wont actually do anything")

    passive_manager = MockedDelegationCommunicator(name="MockedCommunicator", manager_name="MockedManager")

    rospy.spin()



import rospy
from delegation_components.delegation_clients import DelegationClientBase
from delegation_components.delegation_manager import DelegationManager
from task_decomposition_module.msg import CFP
from task_decomposition_module.srv import Precommit, PrecommitResponse, \
    Propose, ProposeResponse, Failure, FailureResponse
from delegation_components.cost_evaluators import CostEvaluatorBase
from delegation_components.goal_wrappers import GoalWrapperBase
from copy import copy


"""
This File just contains classes that are used to mock different objects of the
package, used by tests.
"""


class FunctionPointerTester(object):

    def __init__(self):
        self.function_called = False

    def function(self):
        self.function_called = True


class MockedClient(DelegationClientBase):

    def __init__(self):
        super(MockedClient, self).__init__()
        self.started_working = False

    def delegate(self, goal_name):
        raise RuntimeError("MockedClient cant delegate")

    def start_work(self, delegation_id):
        self.started_working = True


class MockedCostEvaluator(CostEvaluatorBase):

    def __init__(self, cost, possibility):
        super(MockedCostEvaluator, self).__init__()
        self._last_cost = cost
        self._last_possibility = possibility

    # noinspection PyUnusedLocal
    def compute_cost_and_possibility(self, goal_representation, current_task_count, max_task_count, current_depth, max_depth, members, own_name):

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

    def check_if_still_alive(self):
        return True


class MockedDelegationManager(object):

    def __init__(self):
        self._name = "test_name"
        self.cfe = None
        self.m_name = None
        self.client_id = None
        self.goal_wrapper = None
        self.clients = []
        self.terminated = []
        self.task_ended = [False, ""]
        self.stepped = []
        self.own_cost = None
        self.steps = None
        self.known_depth = None
        self.start_service_prefix = None

    def add_client(self, client_id):
        self.clients.append(client_id)

    def remove_client(self, client_id):
        self.clients.remove(client_id)

    def get_name(self):
        return self._name

    def set_cost_function_evaluator(self, cost_function_evaluator, manager_name, client_id):
        self.cfe = cost_function_evaluator
        self.m_name = manager_name
        self.client_id = client_id

    def delegate(self, goal_wrapper, client_id, known_depth, auction_steps=3, own_cost=-1):
        self.goal_wrapper = goal_wrapper
        self.own_cost = own_cost
        self.steps = auction_steps
        self.client_id = client_id
        self.known_depth = known_depth
        return 1

    def terminate(self, auction_id):
        self.terminated.append(auction_id)

    def end_task(self, goal_name):
        self.task_ended = [True, goal_name]

    def do_step(self, delegation_ids):
        self.stepped = delegation_ids

    @property
    def depth_checking_possible(self):
        return True

    def start_depth_service(self, prefix):
        self.start_service_prefix = prefix


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

    def start_communication(self):
        self._precom_service = rospy.Service(name=self._name + DelegationManager.precom_suffix, service_class=Precommit, handler=self.__precom_callback)
        self._propose_service = rospy.Service(name=self._name + DelegationManager.propose_suffix, service_class=Propose, handler=self.__propose_callback)
        self._failure_service = rospy.Service(name=self._name + DelegationManager.failure_suffix, service_class=Failure, handler=self.__failure_callback)

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

    def send_precom(self, target_name, auction_id, proposal_value, goal_representation, goal_name, depth, delegation_members):

        service_name = target_name + DelegationManager.precom_suffix
        rospy.wait_for_service(service=service_name, timeout=DelegationManager.SERVICE_TIMEOUT)

        send_precom = rospy.ServiceProxy(service_name, Precommit)
        response = send_precom(goal_representation, self._name, goal_name, auction_id, proposal_value, depth, delegation_members)

        return response

    def send_failure(self, auctioneer_name, auction_id):

        service_name = auctioneer_name + DelegationManager.failure_suffix
        rospy.wait_for_service(service=service_name, timeout=DelegationManager.SERVICE_TIMEOUT)

        send_failure = rospy.ServiceProxy(service_name, Failure)
        response = send_failure(self._name, auction_id)
        return response

    def send_cfp(self, goal_representation, auction_id, depth, delegation_members):

        msg = CFP()
        msg.goal_representation = goal_representation
        msg.name = self._name
        msg.auction_id = auction_id
        msg.depth = depth
        msg.current_members = delegation_members

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

    def set_precom_response(self, acceptance, still_bidding, cost):
        self.PAcceptance = acceptance
        self.PBidding = still_bidding
        self.PNewCost = cost


class MockedManager(object):

    def __init__(self):
        self._prefix = "test_prefix"
        self.plan = dict()
        self.actions = dict()
        self.actions[0] = "DelegationBehaviour"
        self.actions[1] = "BehaviourBase"
        self.plan["actions"] = self.actions
        self.plan["cost"] = 2.0
        b1 = MockedBehaviour(name="DelegationBehaviour", b_type="Delegation")
        b2 = MockedBehaviour(name="BehaviourBase", b_type="Base")
        b3 = MockedBehaviour(name="DelegationBehaviour2", b_type="Delegation")
        self.behaviours = [b1, b2, b3]
        self.failing_plans = False
        self.plan_exception = False

    # noinspection PyUnusedLocal
    def plan_with_additional_goal(self, goal_statement):
        if self.plan_exception:
            raise Exception()
        if self.failing_plans:
            return dict()
        plan = dict()
        actions = copy(self.actions)
        actions[2] = "DelegationBehaviour2"
        actions[3] = "BehaviourBase"
        plan["actions"] = actions
        plan["cost"] = 4.0
        return plan

    # noinspection PyUnusedLocal
    def plan_this_single_goal(self, goal_statement):
        if self.plan_exception:
            raise Exception()
        if self.failing_plans:
            return dict()
        plan = dict()
        actions = dict()
        actions[0] = "BehaviourBase"
        actions[1] = "DelegationBehaviour2"
        plan["actions"] = actions
        plan["cost"] = self.plan["cost"]
        return plan

    @property
    def prefix(self):
        return self._prefix


class MockedBehaviour(object):

    def __init__(self, name, b_type):
        self.name = name
        self.behaviour_type = b_type

"""
This File contains classes that are used to mock different objects of the
package. Theses mocked objects are used for testing purposes.

@author: Mengers
"""

import rospy
from delegation_components.delegation_clients import DelegationClientBase
from delegation_components.delegation_manager import DelegationManager
from delegation_module.msg import CFP
from delegation_module.srv import Precommit, PrecommitResponse, \
    Propose, ProposeResponse, Failure, FailureResponse
from delegation_components.cost_evaluators import CostEvaluatorBase
from delegation_components.goal_wrappers import GoalWrapperBase


class MockedClient(DelegationClientBase):
    """
    Mocked DelegationClient
    """

    def __init__(self):
        super(MockedClient, self).__init__()
        self.started_working = False
        self.successful_id = -1
        self.goal_name = None
        self.conditions = None
        self.satisfaction_t = None
        self.success_func = None
        self._manager_active = True
        self.step_done = False
        self.terminated = False
        self.own_cost = None
        self.start_work_func = None

    def delegate(self, goal_name, conditions=None, own_cost=-1,
                 satisfaction_threshold=1.0, success_function=None,
                 start_work_function=None):
        """
        safes the given information
        """
        self.own_cost = own_cost
        self.goal_name = goal_name
        self.conditions = conditions
        self.satisfaction_t = satisfaction_threshold
        self.success_func = success_function
        self.start_work_func = start_work_function

    def start_work_for_delegation(self, delegation_id):
        self.started_working = True

    def delegation_successful(self, delegation_id):
        self.successful_id = delegation_id

    def do_step(self):
        self.step_done = True

    def terminate_all_delegations(self):
        self.terminated = True

    def toggle_manager_active(self):
        self._manager_active = not self._manager_active


class MockedCostEvaluator(CostEvaluatorBase):
    """
    Mocked CostEvaluator
    """

    def __init__(self, cost, possibility):
        super(MockedCostEvaluator, self).__init__()
        self._last_cost = cost
        self._last_possibility = possibility

    # noinspection PyUnusedLocal
    def compute_cost_and_possibility(self, goal_representation, current_task_count, max_task_count, current_depth, max_depth, members, own_name):
        """
        Returns the cost and possibility set at construction, parameters do not
        matter

        :return: set cost, possibility
        :rtype: float, bool
        """

        return self._last_cost, self._last_possibility

    def _plan_and_extract_parameters(self, goal_representation, own_name, members, depth, max_depth, task_count, max_task_count):
        raise RuntimeError("This should not be used with the MockedCostEvaluator")


class MockedGoalWrapper(GoalWrapperBase):
    """
    Mocked GoalWrapper
    """

    def __init__(self, name):
        super(MockedGoalWrapper, self).__init__(name=name)
        self._manager_name = ""

    def send_goal(self, name):
        self._manager_name = name
        self._created_goal = True

    def get_manager(self):
        """
        Returns the name set at construction

        :return: set manager_name
        :rtype: str
        """

        return self._manager_name

    def terminate_goal(self):
        self._created_goal = False

    def get_goal_representation(self):
        return self._name

    def check_if_still_alive(self):
        """
        Returns always True

        :return: True
        :rtype: bool
        """

        return True

    def check_goal_finished(self):
        """
        Returns always True

        :return: True
        :rtype: bool
        """

        return True


class MockedDelegationManager(object):
    """
    Mocked DelegationManager without ros communication
    """

    def __init__(self):
        self._name = "test_name"
        self.cfe = None
        self.agent_name = None
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
        self.cost_computable = None
        self.failed_goals = []

    def add_client(self, client_id):
        self.clients.append(client_id)

    def remove_client(self, client_id):
        self.clients.remove(client_id)

    @property
    def name(self):
        return self._name

    def set_cost_function_evaluator(self, cost_function_evaluator, agent_name, client_id):
        self.cfe = cost_function_evaluator
        self.agent_name = agent_name
        self.client_id = client_id

    def remove_cost_function_evaluator(self):
        self.cfe = None
        self.agent_name = None
        self.client_id = None

    def delegate(self, goal_wrapper, client_id, known_depth, auction_steps=DelegationManager.DEFAULT_AUCTION_STEPS, own_cost=-1):
        """
        Mocked delegate

        :return: delegation id of 1
        :rtype: int
        """

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
        """
        Returns True

        :return: True
        :rtype: bool
        """

        return True

    def start_depth_service(self, prefix):
        self.start_service_prefix = prefix

    def fail_task(self, goal_name):
        self.failed_goals.append(goal_name)


class MockedDelegationCommunicator(object):
    """
    Mocks the ROS communication of the DelegationManager
    """

    def __init__(self, name, manager_name):
        """
        Constructor, starts Topics

        Services hve to be started separately
        :param name: used name of this "DelegationManager"
        :param manager_name: used name of agent
        """

        self._name = name
        self._manager_name = manager_name
        self._service_prefix = manager_name + '/'
        self._cfp_publisher = rospy.Publisher(name=DelegationManager.cfp_topic_name, data_class=CFP, queue_size=10)
        self._cfp_subscriber = rospy.Subscriber(name=DelegationManager.cfp_topic_name, data_class=CFP, callback=self._cfp_callback)
        self._precom_service = rospy.Service(name=self._name + DelegationManager.precom_suffix, service_class=Precommit, handler=self._precom_callback)
        self._propose_service = rospy.Service(name=self._name + DelegationManager.propose_suffix, service_class=Propose, handler=self._propose_callback)
        self._failure_service = rospy.Service(name=self._name + DelegationManager.failure_suffix, service_class=Failure, handler=self._failure_callback)
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
        """
        Starts all Services
        """

        self._precom_service = rospy.Service(name=self._name + DelegationManager.precom_suffix, service_class=Precommit, handler=self._precom_callback)
        self._propose_service = rospy.Service(name=self._name + DelegationManager.propose_suffix, service_class=Propose, handler=self._propose_callback)
        self._failure_service = rospy.Service(name=self._name + DelegationManager.failure_suffix, service_class=Failure, handler=self._failure_callback)

    def __del__(self):
        self.stop_communication()
        self._cfp_publisher.unregister()
        self._cfp_subscriber.unregister()

    def stop_communication(self):
        """
        Terminates all services
        """

        self._precom_service.shutdown()
        self._propose_service.shutdown()
        self._failure_service.shutdown()

    def _precom_callback(self, request):

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

    def _propose_callback(self, request):

        print("Got Propose Request")

        self.Pro_last = request
        self.got_pro = True

        return ProposeResponse()

    def _failure_callback(self, request):

        print("Got Failure Request")

        self.Fail_last = request
        self.got_fail = True

        return FailureResponse()

    def _cfp_callback(self, msg):

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

    #
    #   Different Message interaction
    #

    def reset_messages(self):
        """
        Resets message received values
        """

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
        """
        Sets sent PRECOM answer on PRECOM call
        :param acceptance: acceptance
        :param still_bidding: still_bidding
        :param cost: cost
        """

        self.PAcceptance = acceptance
        self.PBidding = still_bidding
        self.PNewCost = cost

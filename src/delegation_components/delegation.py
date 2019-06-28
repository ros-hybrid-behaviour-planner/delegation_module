"""
Delegation and objects used by it, especially the Proposal

@author: Mengers, Hrabia
"""

import bisect
import math
from delegation_errors import DelegationContractorError
from cost_evaluators import CostEvaluatorBase

import rospy


class Delegation(object):
    """
    Class that represents a possible delegation of a goal at any state.
    It contains all necessary information about this delegation.
    """

    def __init__(self, goal_wrapper, auction_id, client_id, depth=0, auction_steps=1, max_timeout_steps=2):
        """
        Constructor

        :param goal_wrapper: GoalWrapper for the goal that should be delegated
        :type goal_wrapper: GoalWrapperBase
        :param auction_id: ID of the delegation/auction
        :type auction_id: int
        :param client_id: ID of the client that started this delegation
        :type client_id: int
        :param depth: depth of this delegation
        :type depth: int
        :param auction_steps: steps that are waited for proposals
        :type auction_steps: int
        :param max_timeout_steps: maximum number of consecutive timeouts for
                this delegation
        :type max_timeout_steps: int
        """

        self._goal_wrapper = goal_wrapper
        self._auction_id = auction_id
        self._client_id = client_id
        self._auction_steps_max = auction_steps
        self._auction_steps = auction_steps
        self._proposals = []
        self._forbidden_bidders = []
        self._contractor = ""
        self._got_contractor = False
        self.state = DelegationState()
        self._depth = depth
        self._max_timeout_steps = max_timeout_steps
        self._timeout_steps = 0

    def __del__(self):
        """
        Destructor for the delegation
        """

        del self._proposals[:]
        del self._forbidden_bidders[:]
        del self.state
        del self._goal_wrapper

    # ------- Manipulating and checking auction steps -------

    def decrement_and_check_steps(self):
        """
        Decrements current auction_steps and checks if all waiting steps are
        finished

        :return: Whether the auction has waited all steps or not
        :rtype: bool
        """

        if self._auction_steps > 0:
            self._auction_steps -= 1

        return self._auction_steps <= 0

    def end_auction_next_step(self):
        """
        Sets the auction_steps to 1 so that the auction will end in the next
        call of DelegationManager.do_step()
        """

        self._auction_steps = 1

    def reset_steps(self):
        """
        Resets the current auction_steps to the defined max-value
        """

        self._auction_steps = self._auction_steps_max

    # ------- Proposals -------

    def add_proposal(self, proposal):
        """
        Adds a proposal to the sorted list of proposals of this delegation

        :param proposal: a proper proposal
        :type proposal: Proposal
        :raises Warning: if bidder of this proposal is forbidden
        """

        name = proposal.name
        if self._forbidden_bidders.__contains__(name):
            raise Warning("Proposal wont be added, bidder " + str(name) +
                          " is forbidden")

        bisect.insort(a=self._proposals, x=proposal)

    def remove_proposal(self, proposal):
        """
        Removes the given proposal for this delegation,
        the list of proposals stays sorted

        :param proposal: The Proposal that should be removed
        :type proposal: Proposal
        """

        self._proposals.remove(proposal)

    def get_best_proposal(self):
        """
        Returns the currently best proposal of this delegation

        :return: best proposal
        :rtype: Proposal
        :raises LookupError: if got no proposals
        """

        if not self.has_proposals():
            raise LookupError("Got no proposals")

        best_proposal = self._proposals[0]

        if math.isnan(best_proposal.value):  # == CostEvaluatorBase.IMPOSSIBLE_COSTS:
            raise LookupError("Got only impossible proposals")

        return best_proposal

    def reset_proposals(self):
        """
        Removes all given proposals for this delegation
        """

        del self._proposals[:]

    def has_proposals(self):
        """
        Returns if the auction of this delegation has any proposals

        :return: whether this delegation has proposals or not
        :rtype: bool
        """

        if len(self._proposals) > 0:
            return True
        return False

    # ------- Forbidding and allowing bidders (not currently used) -------

    def forbid_bidder(self, name):
        """
        Puts this name on the forbidden list, no proposals from this source
        allowed

        :param name: name of the forbidden bidder
        :type name: str
        """

        self._forbidden_bidders.append(name)

    def is_forbidden(self, name):
        """
        Returns whether the name is on the forbidden list or not

        :param name: name of the bidder
        :type name: str
        :return: whether the name is on the forbidden list or not
        :rtype: bool
        """

        return self._forbidden_bidders.__contains__(name)

    def reset_forbidden(self):
        """
        Resets the list of forbidden bidders
        """

        self._forbidden_bidders = []

    def allow_bidder(self, name):
        """
        Allows bidder if he was formerly forbidden, else does nothing

        :param name: name of the bidder
        :type name: str
        """

        if self._forbidden_bidders.__contains__(name):
            self._forbidden_bidders.remove(name)

    # ------- Contractors -------

    def set_contractor(self, name):
        """
        Sets contractor for this delegation,
        changes state of this delegation to delegated

        :param name: name of the contractor
        :type name: str
        :raises DelegationContractorError: if contractor already chosen
        """

        if self._got_contractor:
            raise DelegationContractorError("Contractor already chosen")

        self._contractor = name
        self._got_contractor = True
        self.state.set_delegated_running()

    def get_contractor(self):
        """
        Gets the name of the contractor of this delegation

        :return: name of the contractor
        :rtype: str
        :raises DelegationContractorError: if no contractor is already chosen
        """

        if not self._got_contractor:
            raise DelegationContractorError("No contractor chosen")

        return self._contractor

    def remove_contractor(self):
        """
        Removes the contractor if set
        """

        self._contractor = ""
        self._got_contractor = False

    # ------- Goal -------

    def get_goal_representation(self):
        """
        Gets the Representation of the goal of this delegation

        :return: the representation of the goal
        :rtype: str
        """

        return self._goal_wrapper.get_goal_representation()

    def get_goal_name(self):
        """
        Returns the name of the affiliated goal

        :return: name of the goal
        :rtype: str
        """

        return self._goal_wrapper.goal_name

    def send_goal(self, name):
        """
        Sends the goal to the agent with that name

        If a Exception is raised in the goalwrapper it will be forwarded

        :param name: name of the manager (particular usage depends on system,
                for rhbp use planner_prefix of the manager)
        :type name: str
        """

        self._goal_wrapper.send_goal(name=name)

    def terminate_goal(self):
        """
        Terminates the goal

        If a Exception is raised in the goalwrapper it will be forwarded
        """

        self._goal_wrapper.terminate_goal()

    def check_if_goal_finished(self):
        """
        Checks whether the goal is finished or not

        If a Exception is raised in the goalwrapper it will be forwarded

        :return: whether the goal is finished or not
        :rtype: bool
        """

        return self._goal_wrapper.check_goal_finished()

    # ------- Auction and Contract -------

    def start_auction(self):
        """
        Notifies this delegation that the auction has started, resets old
        proposals and auction_steps
        """

        self.state.set_waiting_for_proposal()
        self.reset_steps()
        self.reset_proposals()

    def make_contract(self, bidder_name, manager_name):
        """
        Makes a contract by setting the bidder as contractor and sending the
        goal to the manager

        Bidder and manager can be the same or different by name

        :param bidder_name: name of the bidder
        :type bidder_name: str
        :param manager_name: name of the manager
        :type manager_name: str
        :raises DelegationContractorError: if a contractor is already chosen
        :raises DelegationError: if the goal could not be sent
        """

        self.set_contractor(name=bidder_name)
        self.send_goal(name=manager_name)

    def check_if_alive(self):
        """
        Checks whether this delegation and its goal are still alive

        If an exception is raised in the goalwrapper it will be forwarded

        :return: whether this delegation and its goal are still alive
        :rtype: bool
        """

        if self._goal_wrapper.check_if_still_alive():
            self._timeout_steps = 0
            return True

        self._timeout_steps += 1

        if self._timeout_steps >= self._max_timeout_steps:
            return False
        else:
            return True

    def terminate_contract(self):
        """
        Removes contractor and terminates goal

        State wont be changed, use finish_delegation instead
        """

        self.remove_contractor()
        self.terminate_goal()

    def finish_delegation(self):
        """
        Removes contractor, terminates goal and resets proposals as well as
        the state of the delegation to finished
        """

        self.terminate_contract()
        self.reset_proposals()
        self.state.set_finished()

    def fail_current_delegation(self):
        """
        Fails current delegation, changes state back to ready,
        removes contractor
        """

        self.terminate_contract()
        self.state.set_ready()

    # ------- Properties -------

    @property
    def auction_id(self):
        """
        Gets the ID of this delegation and the corresponding auction

        :return: the ID
        :rtype: int
        """

        return self._auction_id

    @property
    def client_id(self):
        """
        ID of the client that started this delegation

        :return: ID of the client that started this delegation
        :rtype: int
        """

        return self._client_id

    @property
    def depth(self):
        """
        Depth of this delegation at the level of this agent

        :return: depth
        :rtype: int
        """

        return self._depth

    @property
    def number_of_proposals(self):
        """
        Return the number of returned proposals for the auction
        :return: number of proposals
        """
        return len(self._proposals)


class Proposal(object):
    """
    This is a simple container for a proposal for the auction.
    Contains name of the bidder and the proposed value.
    """

    def __init__(self, name, value):
        """
        Constructor of an instance of Proposal

        :param name: the name of the bidder who made the proposal
        :type name: str
        :param value: the proposed cost
        :type value: float
        """

        self._name = name
        self._value = value

    def __cmp__(self, other):
        """
        Comparator for proposals
        """
        if math.isnan(self.value) and math.isnan(other.value):
            return 0
        if math.isnan(self.value):
            return 1
        if math.isnan(other.value):
            return -1

        return cmp(self.value, other.value)

    def __repr__(self):
        """
        Representation of the proposal

        :return: string that represents the proposal
        :rtype: str
        """

        return "(" + str(self._name) + ": " + str(self._value) + ")"

    @property
    def value(self):
        """
        Returns the value of this proposal

        :return: value of the proposal
        :rtype: float
        """

        return self._value

    @property
    def name(self):
        """
        Returns name of the bidder, that made this proposal

        :return: name of the bidder
        :rtype: str
        """

        return self._name


class DelegationState(object):
    """
    State of a Delegation object.
    Simple Wrapper for the state, that can be:
    READY, WAITING_FOR_PROPOSALS, DELEGATED_RUNNING, FINISHED
    """

    def __init__(self):
        """
        Constructor of the DelegationState,
        starts always in READY
        """

        self._state_id = 0

    def __repr__(self):
        """
        Representation of the DelegationState

        :return: string, that represents the state
        """

        if self._state_id == 0:
            return "READY"
        elif self._state_id == 1:
            return "WAITING_FOR_PROPOSALS"
        elif self._state_id == 2:
            return "DELEGATED_RUNNING"
        elif self._state_id == 3:
            return "FINISHED"
        else:
            return "UNSPECIFIED"

    def is_ready(self):
        """
        Checks if the state is READY

        :return: whether the state is READY or not
        :rtype: bool
        """

        return self._state_id == 0

    def is_waiting_for_proposals(self):
        """
        Checks if the state is WAITING_FOR_PROPOSALS

        :return: whether the state is WAITING_FOR_PROPOSALS or not
        :rtype: bool
        """

        return self._state_id == 1

    def is_delegated_running(self):
        """
        Checks if the state is DELEGATED_RUNNING

        :return: whether the state is DELEGATED_RUNNING or not
        :rtype: bool
        """

        return self._state_id == 2

    def is_finished(self):
        """
        Checks if the state is FINISHED

        :return: whether the state is FINISHED or not
        :rtype: bool
        """

        return self._state_id == 3

    def set_ready(self):
        """
        Sets state to READY
        """

        self._state_id = 0

    def set_waiting_for_proposal(self):
        """
        Sets state to WAITING_FOR_PROPOSALS
        """

        self._state_id = 1

    def set_delegated_running(self):
        """
        Sets state to DELEGATED_RUNNING
        """

        self._state_id = 2

    def set_finished(self):
        """
        Sets state to FINISHED
        """

        self._state_id = 3

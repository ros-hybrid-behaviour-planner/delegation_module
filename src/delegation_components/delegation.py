
import bisect
from delegation_errors import DelegationContractorError


class Delegation(object):
    """
    Class that represents a possible delegation of a goal.
    It contains all necessary information about this delegation.
    """

    def __init__(self, goal_wrapper, auction_id, auction_steps=3):
        """
        Constructor of an instance of Delegation

        :param auction_steps: steps that are waited for proposals
        :param goal_wrapper: goal that should be delegated
        :param auction_id: ID of the delegation/auction
        """

        self.__goal_wrapper = goal_wrapper
        self.__auction_id = auction_id
        self.__auction_steps_max = auction_steps
        self.__auction_steps = auction_steps
        self.__proposals = []
        self.__forbidden_bidders = []
        self.__contractor = ""
        self.__got_contractor = False
        self.state = DelegationState()

    def __del__(self):
        """
        Destructor for the delegation
        """

        del self.__proposals[:]
        del self.__forbidden_bidders[:]
        del self.state
        del self.__goal_wrapper

    def add_proposal(self, proposal):
        """
        Adds a proposal to the sorted list of proposals of this delegation

        :param proposal: a proper proposal
        :raises Warning: if bidder of this proposal is forbidden
        """

        name = proposal.get_name()
        if self.__forbidden_bidders.__contains__(name):
            raise Warning("Proposal wont be added, bidder " + str(name) +
                          " is forbidden")

        bisect.insort(a=self.__proposals, x=proposal)

    def remove_proposal(self, proposal):
        """
        Removes the given proposal for this delegation,
        the list of proposals stays sorted

        :param proposal: The Proposal that should be removed
        """

        self.__proposals.remove(proposal)

    def get_auction_id(self):
        """
        Gets the ID of this delegation and the corresponding auction

        :return: the ID
        """

        return self.__auction_id

    def decrement_and_check_steps(self):
        """
        Decrements current auction_steps and checks if the all steps are
        finished

        :return: Whether the auction has waited all steps or not
        """

        self.__auction_steps -= 1

        return self.__auction_steps == 0

    def reset_steps(self):
        """
        Resets the current auction_steps to the defined max-value
        """

        self.__auction_steps = self.__auction_steps_max

    def get_goal_representation(self):
        """
        Gets the Representation of the goal of this delegation

        :return: the representation of the goal
        """

        return self.__goal_wrapper.get_goal_representation()

    def get_best_proposal(self):
        """
        Returns the currently best proposal of this delegation

        :return: best proposal
        :raises LookupError: if got no proposals
        """

        if not self.has_proposals():
            raise LookupError("Got no proposals")

        return self.__proposals[0]

    def reset_proposals(self):
        """
        Removes all given proposals for this delegation
        """

        del self.__proposals[:]

    def has_proposals(self):
        """
        Returns if the auction of this delegation has any proposals

        :return: boolean whether it has proposals or not
        """

        if len(self.__proposals) > 0:
            return True
        return False

    def forbid_bidder(self, name):
        """
        Puts this name on the forbidden list, no proposals from this source
        allowed

        :param name: name of the forbidden bidder
        """

        self.__forbidden_bidders.append(name)

    def is_forbidden(self, name):
        """
        Returns whether the name is on the forbidden list or not

        :param name: name of the bidder
        :return: whether the name is on the forbidden list or not
        """

        return self.__forbidden_bidders.__contains__(name)

    def reset_forbidden(self):
        """
        Resets the list of forbidden bidders
        """

        self.__forbidden_bidders = []

    def allow_bidder(self, name):
        """
        Allows bidder if he was formerly forbidden, else does nothing

        :param name: name of the bidder
        """

        if self.__forbidden_bidders.__contains__(name):
            self.__forbidden_bidders.remove(name)

    def set_contractor(self, name):
        """
        Sets contractor for this delegation,
        changes state of this delegation to delegated

        :param name: name of the contractor
        :raises DelegationContractorError: if contractor already chosen
        """

        if self.__got_contractor:
            raise DelegationContractorError("Contractor already chosen")

        self.__contractor = name
        self.__got_contractor = True
        self.state.set_delegated_running()

    def get_contractor(self):
        """
        Gets the name of the contractor of this delegation

        :return: name of the contractor
        :raises DelegationContractorError: if no contractor is already chosen
        """

        if not self.__got_contractor:
            raise DelegationContractorError("No contractor chosen")

        return self.__contractor

    def fail_current_delegation(self):
        """
        Fails current delegation, changes state back to waiting,
        removes contractor
        """

        self.terminate_contract()
        self.state.set_waiting_for_proposal()

    def remove_contractor(self):
        """
        Removes the contractor if set
        """

        self.__contractor = ""
        self.__got_contractor = False

    def get_goal_name(self):
        """
        Returns the name of the affiliated goal

        :return: name of the goal
        :rtype: str
        """

        return self.__goal_wrapper.get_goal_name()

    def send_goal(self, name):
        """
        Sends goal to manager with that name

        If a Exception is raised in the goalwrapper it will be forwarded

        :param name: name of the manager (particular usage depends on system,
                for rhbp use planner_prefix of the manager)
        :type name: str
        """

        self.__goal_wrapper.send_goal(name=name)

    def terminate_contract(self):
        """
        Removes contractor and terminates goal
        """

        self.remove_contractor()
        self.__goal_wrapper.terminate_goal()

    def finish_delegation(self):

        self.terminate_contract()
        self.reset_proposals()
        self.state.set_finished()


class Proposal(object):
    """
    This is a simple container for a proposal for an auction.
    Contains name of the bidder and the proposed value.
    """

    def __init__(self, name, value):
        """
        Constructor of an instance of Proposal

        :param name: the name of the bidder who made the proposal
        :param value: the proposed cost
        """

        self.__name = name
        self.__value = value

    def __cmp__(self, other):
        """
        Comparator for proposals
        """

        return cmp(self.get_value(), other.get_value())

    def __repr__(self):
        """
        Representation of the proposal

        :return: string that represents the proposal
        """

        return "("+str(self.__name)+", "+str(self.__value)+")"

    def get_value(self):
        """
        Returns the value of this proposal

        :return: value of the proposal
        """

        return self.__value

    def get_name(self):
        """
        Returns name of the bidder, that made this proposal

        :return: name of the bidder
        """

        return self.__name


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

        self.__state_id = 0

    def __repr__(self):
        """
        Representation of the DelegationState

        :return: string, that represents the state
        """

        if self.__state_id == 0:
            return "READY"
        elif self.__state_id == 1:
            return "WAITING_FOR_PROPOSALS"
        elif self.__state_id == 2:
            return "DELEGATED_RUNNING"
        elif self.__state_id == 3:
            return "FINISHED"
        else:
            return "UNSPECIFIED"

    def is_ready(self):
        """
        Checks if the state is READY

        :return: whether the state is READY or not
        """

        return self.__state_id == 0

    def is_waiting_for_proposals(self):
        """
        Checks if the state is WAITING_FOR_PROPOSALS

        :return: whether the state is WAITING_FOR_PROPOSALS or not
        """

        return self.__state_id == 1

    def is_delegated_running(self):
        """
        Checks if the state is DELEGATED_RUNNING

        :return: whether the state is DELEGATED_RUNNING or not
        """

        return self.__state_id == 2

    def is_finished(self):
        """
        Checks if the state is FINISHED

        :return: whether the state is FINISHED or not
        """

        return self.__state_id == 3

    def set_ready(self):
        """
        Sets state to READY
        """

        self.__state_id = 0

    def set_waiting_for_proposal(self):
        """
        Sets state to WAITING_FOR_PROPOSALS
        """

        self.__state_id = 1

    def set_delegated_running(self):
        """
        Sets state to DELEGATED_RUNNING
        """

        self.__state_id = 2

    def set_finished(self):
        """
        Sets state to FINISHED
        """

        self.__state_id = 3

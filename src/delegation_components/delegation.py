
import bisect


class Delegation(object):
    """
    Class that represents a possible delegation of a goal.
    It contains all necessary information about this delegation.
    """

    def __init__(self, goal, auction_id):
        """
        Constructor of an instance of Delegation

        :param goal: goal that should be delegated      TODO possibly subject to change
        :param auction_id: ID of the delegation/auction
        """
        self.__goal = goal
        self.__auction_id = auction_id
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
        # TODO delete goal, depending on implementation

    def add_proposal(self, proposal):
        """
        Adds a proposal to the sorted list of proposals of this delegation

        :param proposal: a proper proposal
        :raises Warning: if bidder of this proposal is forbidden
        """

        if self.__forbidden_bidders.__contains__(proposal.get_name()):
            raise Warning("Proposal wont be added, bidder is forbidden")

        bisect.insort(self.__proposals, proposal)

    def remove_proposal(self, proposal):
        """
        Removes the given proposal for this delegation

        :param proposal: The Proposal that should be removed
        """
        self.__proposals.remove(proposal)

    def get_auction_id(self):
        """
        Gets the ID of this delegation and the corresponding auction

        :return: the ID
        """

        return self.__auction_id

    def get_goal_representation(self):
        """
        Gets the Representation of the goal of this delegation

        :return: the representation of the goal
        """

        # TODO get PDDL info of goal
        pddlstring = ""     # placeholder

        return pddlstring

    def get_best_proposal(self):
        """
        Returns the currently best proposal of this delegation

        :return: best proposal
        :raises if got no proposals
        """

        if not self.has_proposals():
            raise NameError("Got no proposals")

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
        :raises NameError: if contractor already chosen
        """

        if self.__got_contractor:
            raise NameError("Contractor already chosen")

        self.__contractor = name
        self.__got_contractor = True
        self.state.set_delegated_running()

    def get_contractor(self):
        """
        Gets the name of the contractor of this delegation

        :return: name of the contractor
        :raises NameError: if no contractor is already chosen
        """

        if not self.__got_contractor:
            raise NameError("No contractor chosen")

        return self.__contractor

    def fail_current_delegation(self):
        """
        Fails current delegation, changes state back to waiting,
        removes contractor
        """

        self.__contractor = ""
        self.__got_contractor = False
        self.state.set_waiting_for_proposal()


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



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
        self.__contractor = ""
        self.__state = 1  # TODO states of a delegation

    def add_proposal(self, proposal):
        """
        Adds a proposal to the sorted list of proposals of this delegation

        :param proposal: a proper proposal
        """
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
        """
        if len(self.__proposals) == 0:
            # TODO raise Exception
            print "Got no Proposals"
        return self.__proposals[0]

    def reset_proposals(self):
        """
        Removes all given proposals for this delegation
        """
        self.__proposals = []
        # TODO myb delete old proposals right

    def has_proposals(self):
        """
        Returns if the auction of this delegation has any proposals

        :return: boolean whether it has proposals or not
        """
        if len(self.__proposals) > 0:
            return True
        return False

    def set_contractor(self, name):
        """
        Sets contractor for this delegation

        :param name: name of the contractor
        """
        self.__contractor = name
        # TODO change state of delegation

    def get_contractor(self):
        """
        Gets the name of the contractor of this delegation

        :return: name of the contractor
        """
        # TODO check if a contractor is already chosen
        return self.__contractor


class Proposal(object):
    """
    This is a simple container for a proposal for an auction
    Contains name of the bidder and the proposed value
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
        return cmp(self.get_value(), other.get_value())

    def __repr__(self):
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



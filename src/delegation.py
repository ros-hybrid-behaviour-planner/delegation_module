#! /usr/bin/env python2


import bisect


class Delegation(object):

    def __init__(self, goal):
        self.__goal = goal
        self.__proposals = []
        self.__state = 1  # TODO

    def add_proposal(self, proposal):
        """
        Adds a proposal to the list of proposals of this delegation
        :param proposal: a proper proposal
        """
        bisect.insort(self.__proposals, proposal)

    def get_pddl(self):

        # return goal pddl TODO
        pass

    def get_best_proposal(self):
        """
        Returns the currently best proposal
        :return: best proposal
        """
        return self.__proposals[0]


class Proposal(object):
    """
    This is a simple container for a proposal
    """

    def __init__(self, name, value):
        """
        Constructor
        :param name: the name of the bidder who gave the proposal
        :param value: the proposed cost
        """
        self.__name = name
        self.__value = value

    def __cmp__(self, other):
        return cmp(self.get_value(), other.get_value())

    def __repr__(self):
        return "("+str(self.__name)+", "+str(self.__value)+")"

    def get_value(self):
        return self.__value

    def get_name(self):
        return self.__name



"""
Task as representation of a Delegation from a different agent

@author: Mengers
"""


class Task(object):
    """
    Class that represents a task in context of the DelegationManager
    """

    def __init__(self, auction_id, auctioneer_name, goal_name, depth, employers=None):
        """
        Constructor

        :param auction_id: ID of the corresponding auction
        :type auction_id: int
        :param auctioneer_name: name of the auctioneer (my employer)
        :type auctioneer_name: str
        :param goal_name: name of the corresponding goal (in the RHBP Manager)
        :type goal_name: str
        :param depth: depth of this task
        :type depth: int
        :param employers: all employers (treelike) of this task
        :type employers: list(str)
        """

        self._goal_name = goal_name
        self._auction_id = auction_id
        self._auctioneer_name = auctioneer_name
        self._depth = depth
        if employers is None:
            self._employers = []
        else:
            self._employers = employers

    @property
    def auction_id(self):
        """
        The ID of the auction corresponding to this task

        :return: the auction ID
        :rtype: int
        """

        return self._auction_id

    @property
    def auctioneer_name(self):
        """
        Gets the name of the auctioneer (my employer)

        :return: name of the auctioneer
        :rtype: str
        """

        return self._auctioneer_name

    @property
    def goal_name(self):
        """
        The name of corresponding goal

        :return: name of the goal
        :rtype: str
        """

        return self._goal_name

    @property
    def depth(self):
        """
        Current Depth of this task

        :return: depth of this task
        :rtype: int
        """

        return self._depth

    @property
    def employers(self):
        """
        Returns all employers (treelike) of this task

        :return: all employers (treelike) of this task
        :rtype: list(str)
        """

        return self._employers



class Task(object):
    """
    Class that represents a task in context of the DelegationManager
    """

    def __init__(self, auction_id, auctioneer_name, goal_name):
        """
        Constructor

        :param auction_id: ID of the corresponding auction
        :type auction_id: int
        :param auctioneer_name: name of the auctioneer (my employer)
        :type auctioneer_name: str
        :param goal_name: name of the corresponding goal (in the RHBP Manager)
        :type goal_name: str
        """

        self.__goal_name = goal_name
        self.__auction_id = auction_id
        self.__auctioneer_name = auctioneer_name

    def get_auction_id(self):
        """
        Gets the ID of the auction corresponding to this task

        :return: the auction ID
        :rtype: int
        """

        return self.__auction_id

    def get_auctioneer_name(self):
        """
        Gets the name of the auctioneer (my employer)

        :return: name of the auctioneer
        :rtype: str
        """

        return self.__auctioneer_name

    def goal_name(self):
        """
        Gets the name of corresponding goal

        :return: name of the goal
        :rtype: str
        """

        return self.__goal_name

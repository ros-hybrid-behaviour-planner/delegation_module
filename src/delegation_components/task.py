
from copy import copy


class Task(object):
    """
    Class that represents a task in context of the DelegationManager
    """

    def __init__(self, auction_id, auctioneer_name, goal_name, employer_incidence=None):
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

        if employer_incidence is None:
            self.__employer_incidence = {}
        else:
            self.__employer_incidence = employer_incidence

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

    @property
    def employer_incidence(self):
        return self.__employer_incidence


class EmployerAdministration(object):

    def __init__(self):
        self._employer_dict = {}

    def add_task(self, task):
        new_incidence = task.employer_incidence
        for employer in new_incidence.keys():
            if self._employer_dict.__contains__(employer):
                self._employer_dict[employer] += new_incidence[employer]
            else:
                self._employer_dict[employer] = new_incidence[employer]

        return copy(self._employer_dict)

    def remove_task(self, task):
        old_incidence = task.employer_incidence
        for employer in old_incidence.keys():
            if self._employer_dict.__contains__(employer):
                combined_value = self._employer_dict[employer]
                remove_value = old_incidence[employer]
                if combined_value > remove_value:
                    self._employer_dict[employer] = combined_value - remove_value
                else:
                    del self._employer_dict[employer]

        return copy(self._employer_dict)

    def get_current_employer_incidence(self):
        return copy(self._employer_dict)

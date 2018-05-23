#! /usr/bin/env python2


import rospy
from behaviour_components.goals import Goal
from delegation import Delegation


class DelegationManager(object):

    def __init__(self, name="", topic_name="CFP_topic"):
        self._name = name
        self._topic_name = topic_name
        self.__delegations = []
        self.__active_delegated_task = False
        self.__delegated_task = None

        self.__init_services()
        self.__init_topics()

    def __init_topics(self):
        """
        Initiates needed subscribers and publishers
        """
        self._cfp_publisher = rospy.Publisher(self._topic_name, "TODO")
        self._cfp_subscriber = rospy.Subscriber(self._topic_name, "TODO", self.__cfp_callback)

    def __init_services(self):
        """
        Initiates needed services
        """
        self._precom_service = rospy.Service(self._name+"/precom", "TODO", self.__precom_callback)
        self._propose_service = rospy.Service(self._name+"/propose", "TODO", self.__propose_callback)
        self._failure_service = rospy.Service(self._name+"/failure", "TODO", self.__failure_callback)
        self._terminate_service = rospy.Service(self._name+"/terminate", "TODO", self.__terminate_callback)

    def __terminate_callback(self, request):
        # stop currently active delegated task if present TODO
        pass

    def __precom_callback(self, request):

        # check if proposal still accurate TODO
        # answer with new proposal if not
        # else reassure
        pass

    def __propose_callback(self, request):
        # find right delegation TODO
        # add proposal to list TODO
        pass

    def __failure_callback(self, request):
        # start new delegation TODO
        pass

    def __cfp_callback(self, msg):

        # compute cost TODO
        cost = 5
        # if possible send proposal TODO

    def delegate(self, goal):
        new = Delegation(goal)

        self.__delegations.append(new)

        msg = "TODO"    # TODO

        self._cfp_publisher.publish(msg)

    def make_step(self):
        # do work TODO
        pass


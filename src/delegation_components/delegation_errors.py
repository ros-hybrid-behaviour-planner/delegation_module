

class DelegationServiceError(Exception):
    """
    Error that represents any problem with the sending of a
    ServiceCall/TopicMessage
    """
    pass


class DelegationPlanningWarning(Warning):
    """
    Warning that shows that something went wrong with planning and hands down
    the problem information (mainly for debugging purposes)
    """
    pass

class DelegationContractorError(Exception):
    """
    Error that shows that there is a problem with getting/setting the
    contractor of a delegation
    """
    pass






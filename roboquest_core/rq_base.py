from roboquest_core.rq_node import RQNode


class RQBase(RQNode):
    """
    The node which handles the general data for the RoboQuest application.
    """

    def __init__(self, node_name: str = 'RQBase'):
        super().__init__(node_name)

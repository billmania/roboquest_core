from roboquest_core.rq_node import RQNode
from roboquest_core.rq_hat import RQHAT


class RQManage(RQNode):
    """
    The node which manages the RoboQuest application.
    """

    def __init__(self, node_name: str = 'RQManage'):
        super().__init__(node_name)

        parameters = dict()
        self.hat = RQHAT(parameters)

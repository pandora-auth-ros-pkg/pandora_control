class CostNode(object):

    """ @brief: Basic node in the acyclic directed graph of cost which is
        responsible for calculating a single cost from a certain source of error
        with its own cost function. On its own can describe a raw source of
        error, a 'leaf' of the A.D.G."""

    def __init__(self):
        """ @brief: Initiates a CostNode object"""

        self.__cost = -1.0

    def update_cost(self):
        """ @brief: Implements the cost function with which the estimation of
        cost due to a source of error is updated. Updates self.__cost

        @return: nothing

        """

        self.__cost = 0.0

    def get_cost(self):
        """ @brief: Gets most recent cost calculated from update_error method

        @return: double, the cost of this cost_node

        """

        return self.__cost

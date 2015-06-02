from cost_node import CostNode
from fusion_strategy import FusionStrategy

class FuseCostNode(CostNode):

    """ @brief: A node in the acyclic directed graph of costs which calculates
        a certain cost from a source of error, plus must combine costs from other
        CostNodes with a concrete FusionStrategy"""

    def __init__(self, nodes=None, strategy=None):
        """ @brief: Simple initiation of a FuseCostNode object"""

        super(FuseCostNode, self).__init__()
        self.__inside_cost = -1.0
        if nodes:
            self.lesser_cost_nodes = nodes
        else:
            self.lesser_cost_nodes = list()
        if strategy:
            self.fusion_strategy = strategy
            self.fusion_strategy.set_nodes(self.lesser_cost_nodes)
        else:
            self.fusion_strategy = FusionStrategy()

    def set_lesser_nodes(self, nodes):
        """ @brief: Setter of lesser_cost_nodes variable.
            Should update fusion_strategy afterwards

        @param nodes: lesser cost nodes to be fused by a strategy
        @type nodes: list of CostNode
        @return: nothing

        """
        self.lesser_cost_nodes = nodes
        self.fusion_strategy.set_nodes(self.lesser_cost_nodes)

    def set_fusion_strategy(self, strategy):
        """ @brief: Setter of fusion_strategy variable. Should set it with
            existing lesser_cost_nodes afterwards

        @param strategy: describes a way to fuse costs
        @type strategy: FusionStrategy
        @return: nothing

        """
        self.fusion_strategy = strategy
        self.fusion_strategy.set_nodes(self.lesser_cost_nodes)

    def fuse_cost(self):
        """ @brief: Uses FusionStrategy in order to combine cost from the
            CostNodes that this FuseCostNode object holds.

        Delegate to FusionStrategy

        @return: double, a unified estimation of cost from various
        sources of error

        """
        unified_cost = self.fusion_strategy.fuse()
        return unified_cost

    def update_cost(self):
        """ @brief: First it calculates its own contribution to the costs,
            then fuse_cost method is called to make a unified cost estimation,
            last the process_cost method is called to refine,
            if needed, the resulting cost. Updates self.__cost

        @return: nothing

        """
        self.__inside_update_cost()
        self.__cost = self.fuse_cost()
        self.process_cost()

    def process_cost(self):
        """ @brief: Processes the resulting unified cost estimation

        @return: nothing

        """
        pass

    def __inside_update_cost(self):
        """ @brief: Calculates as a cost function this CostNode object's own
            cost estimation, if it has one. Updates self.__inside_cost

        @return: nothing

        """
        self.__inside_cost = 0.0

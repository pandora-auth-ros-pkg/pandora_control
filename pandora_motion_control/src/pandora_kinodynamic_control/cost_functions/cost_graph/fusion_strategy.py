class FusionStrategy(object):

    """This class represents an abstract way to fuse costs from various sources
       into a unified cost concept. It should be subclassed to be implemented
       properly and used in conjuction with a FuseCostNode object"""

    def __init__(self):
        """Simple initiation of a FusionStrategy object"""
        self.cost_nodes = list()

    def set_nodes(self, nodes):
        """Setter of nodes that will provide the FusionStrategy object with
           the cost to be fused

        @param nodes list, of CostNode objects
        @return: nothing

        """
        self.cost_nodes = nodes

    def fuse_with(self, cost):
        """Fuses costs from self.cost_nodes together considering cost param
           with a certain way. Should be implemented by a subclass

        @param cost double, a cost param to be taken into account when fusing
        the costs from the self.cost_nodes
        @return: double, a unified perception of these cost

        """
        if not self.cost_nodes:
            return cost
        else:
            return max((cost, max(self.cost_nodes)))

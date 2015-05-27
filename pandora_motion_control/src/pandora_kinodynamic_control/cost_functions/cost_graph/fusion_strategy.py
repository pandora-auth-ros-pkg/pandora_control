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

    def fuse(self):
        """Fuses costs from self.cost_nodes together.
           Should be implemented by a subclass.
        @return: double, a unified perception of these costs

        """
        if not self.cost_nodes:
            return 0.0
        else:
            return max(self.cost_nodes)

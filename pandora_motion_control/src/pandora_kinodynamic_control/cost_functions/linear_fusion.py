from cost_graph.fusion_strategy import FusionStrategy

class LinearFusion(FusionStrategy):

    """A FusionStrategy that implements a weighted linear approach to fusing
       costs"""

    def __init__(self):
        """Simple initiation of a LinearFusion object"""
        super(LinearFusion, self).__init__()
        self.cost_weights = list()

    def set_weights(self, weight_list):
        """TODO: Docstring for set_weights.

        @param weight_list list, of doubles, each weight is a multiplier coeff
        of the corresponding (by index) cost given by a cost node
        @return: nothing

        """
        self.cost_weights = weight_list

    def fuse(self):
        """Fuses costs in a linear way. Each cost is multiplied with a weight
        and then are added all together.
        @return: double, a unified perception of these cost

        """
        if len(self.cost_weights) != len(self.cost_nodes):
            print "[LinearFusion] ERROR: Cannot fuse because length of weight\
                    list is not the same as cost node list"
            print "[LinearFusion] WARN: Returning FusionStrategy's default (max)"
            return super(LinearFusion, self).fuse()
        costs = map(lambda x: x.get_cost(), self.cost_nodes)
        return sum(map(lambda x, y: x * y, self.cost_weights, costs))

# Software License Agreement
__version__ = "0.0.1"
__status__ = "Development"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__authors__ = "Tsirigotis Christos"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"


from cost_node import CostNode
from fusion_strategy import FusionStrategy

class FuseCostNode(CostNode):

    """ @brief: A node in the acyclic directed graph of costs which calculates
        a certain cost from a source of error, plus must combine costs from other
        CostNodes with a concrete FusionStrategy"""

    def __init__(self, nodes=None, strategy=None):
        """ @brief: Simple initiation of a FuseCostNode object"""

        super(FuseCostNode, self).__init__()
        self._inside_cost = 0
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
            if needed, the resulting cost. Updates self._cost

        @return: nothing

        """
        # In current Implementation , cost is calculated only from child nodes
        # self._inside_update_cost()
        # self.process_cost()
        self._cost = self.fuse_cost()


    def process_cost(self):
        """ @brief: Processes the resulting unified cost estimation

        @return: nothing

        """
        pass

    def _inside_update_cost(self):
        """ @brief: Calculates as a cost function this CostNode object's own
            cost estimation, if it has one. Updates self._inside_cost

        @return: nothing

        """
        self._inside_cost = 0.0

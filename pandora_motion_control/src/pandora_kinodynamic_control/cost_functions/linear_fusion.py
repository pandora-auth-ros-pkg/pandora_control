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


from cost_graph.fusion_strategy import FusionStrategy

class LinearFusion(FusionStrategy):

    """ @brief: A FusionStrategy that implements a weighted linear approach to
        fusing costs"""

    def __init__(self):
        """ @brief: Simple initiation of a LinearFusion object"""
        super(LinearFusion, self).__init__()
        self.cost_weights = list()

    def set_weights(self, weight_list):
        """ @brief: Setter for the weights of each cost node

        @param weight_list: each weight is a multiplier coeff
        of the corresponding (by index) cost given by a cost node
        @type weight_list: list of double
        @return: nothing

        """
        self.cost_weights = weight_list

    def fuse(self):
        """ @brief: Fuses costs in a linear way. Each cost is multiplied with a
            weight and then are added all together.

        @return: double, a unified perception of these cost

        """
        if len(self.cost_weights) != len(self.cost_nodes):
            print "[LinearFusion] ERROR: Cannot fuse because length of weight\
                    list is not the same as cost node list"
            print "[LinearFusion] WARN: Returning FusionStrategy's default (max)"
            return super(LinearFusion, self).fuse()
        costs = map(lambda x: x.get_cost(), self.cost_nodes)
        return sum(map(lambda x, y: x * y, self.cost_weights, costs))

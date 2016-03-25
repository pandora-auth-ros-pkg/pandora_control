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
__authors__ = ["Tsirigotis Christos", "Zisis Konstantinos"]
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"


from cost_graph.cost_node import CostNode
from pandora_kinodynamic_control.utils import hausdorff_distance

class TrajectoryCost(CostNode):

    """ @brief: Implementation of CostNode for calculate a cost from trajectory
        dissimilarity errors"""

    def __init__(self):
        """ @brief: Initiates a TrajectoryCost object"""
        super(TrajectoryCost, self).__init__()
        self.expected_trajectory = list()
        self.actual_trajectory = list()

    def update_cost(self):
        """ @brief: Uses expected and actual trajectory (list of poses) in order
            to calculate a cost based on their dissimilarity

            Uses a specific (which?) curve similarity algorithm.
            Updates self._cost.

        @return: nothing

        """
        hausdorff_A = hausdorff_distance(self.expected_trajectory,self.actual_trajectory )
        hausdorff_B = hausdorff_distance(self.actual_trajectory,self.expected_trajectory)
        self._cost = max(hausdorff_A,hausdorff_B)

    def append_actual_pose(self, pose):
        """ @brief: Appends an actual pose to the self.actual_trajectory
            discrete curve

            Should be called in respect to the time_granularity set in the
            TrajectoryCost object.

        @param pose: vehicle's actual pose the moment that granularity
        period has ended
        @type pose: tuple of doubles (x, y, yaw)
        @return: nothing

        """
        self.actual_trajectory.append(pose)

    def set_actual_trajectory(self, trajectory):
        """ @brief: Setter for the actual trajectory of the robot
            among the time instance that the movement command was issued
            and the time instance the movement command has finished
            (after a given duration of time)

            To be called in case there is a mechanism that tracks the vehicle's
            trajectory.

        @param trajectory: vehicle's actual trajectory between 2 moments of time
        @type trajectory: list of tuples (x, y, yaw)
        @return: nothing

        """
        self.actual_trajectory = trajectory

    def get_actual_trajectory(self):
        """ @brief: Getter for the actual trajectory done

        @return: list of tuples (x, y, yaw), the actual trajectory

        """
        return self.actual_trajectory

    def extend_expected_trajectory(self, trajectory):
        """ @brief: Extends self.expected_trajectory with  the expected
            trajectory of the robot as calculated by the local planner

        @param trajectory: the expected trajectory
        @type trajectory: list of tuples (x, y, yaw)
        @return: nothing

        """
        self.expected_trajectory.extend(trajectory)

    def get_expected_trajectory(self):
        """ @brief: Getter for the expected trajectory calculated

        @return: list of tuples (x, y, yaw), the expected trajectory

        """
        return self.expected_trajectory

    def clear_trajectories(self):
        """ @brief: Reset actual and expected trajectories to empty lists

        @return: nothing

        """
        self.actual_trajectory = list()
        self.expected_trajectory = list()

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
from geometry_msgs.msg import Pose

from pandora_kinodynamic_control.utils import find_distance
from pandora_kinodynamic_control.params import *

class GoalCost(CostNode):

    """ @brief: Implementation of CostNode for calculating a cost from goal
        based errors"""

    def __init__(self):
        """ @brief: Initiates a GoalCost object"""
        super(GoalCost, self).__init__()
        self.expected_pose = Pose()
        self.actual_pose = Pose()

    def update_cost(self):
        """ @brief: Uses expected final pose (goal) and actual final pose in
            order to calculate a cost. Updates self._cost

        @return: nothing

        """
        # Relative Reward mode:
        point_a = (self.expected_pose[0],self.expected_pose[1])
        point_b = (self.actual_pose[0],self.actual_pose[1])

        diff_distance = find_distance(point_a,point_b)
        diff_angle = abs(self.expected_pose[2]-self.actual_pose[2])

        max_distance = COMMAND_DURATION * LINEAR_LIMITS[1]
        max_angle    = COMMAND_DURATION * ANGULAR_LIMITS[1]

        self._cost = diff_distance/max_distance + diff_angle/max_angle

        # Absolute Rewards:   (BEWARE : requires retuning of cost_to_reward function)
        #self._cost = find_distance(self.expected_pose, self.actual_pose)

    def set_goal_pose(self, pose):
        """ @brief: Sets goal pose, expected pose of vehicle when movement
            command will have finished

        @param pose: expected final vehicle's pose (we care about x, y and yaw)
        @type pose: Pose
        @return: nothing

        """
        self.expected_pose = pose

    def set_actual_pose(self, pose):
        """ @brief: Sets final pose, actual pose of vehicle when movement
            command has finished

        @param pose: actual final vehicle's pose (we care about x, y and yaw)
        @type pose: Pose
        @return: nothing

        """
        self.actual_pose = pose

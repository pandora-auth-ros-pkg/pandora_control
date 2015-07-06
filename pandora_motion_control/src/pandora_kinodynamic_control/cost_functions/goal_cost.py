from cost_graph.cost_node import CostNode
from geometry_msgs.msg import Pose

from pandora_kinodynamic_control.utils import find_distance

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
        self._cost = find_distance(self.expected_pose, self.actual_pose)
        print "Goal Cost = " + str(self.get_cost())

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

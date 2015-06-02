from cost_graph.cost_node import CostNode
from geometry_msgs.msg import Pose
from tf import transformations
import math

def find_distance(pose_a, pose_b):
    """ @brief: finds distance between two poses in terms of x, y and yaw

    @return: double, euclidean distance of (x, y, yaw)

    """

    quaternion = (
        pose_a.orientation.x,
        pose_a.orientation.y,
        pose_a.orientation.z,
        pose_a.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_a = euler[2]
    quaternion = (
        pose_b.orientation.x,
        pose_b.orientation.y,
        pose_b.orientation.z,
        pose_b.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_b = euler[2]

    x_diff = pose_a.position.x - pose_b.position.x
    y_diff = pose_a.position.y - pose_b.position.y
    yaw_diff = yaw_a - yaw_b

    distance = math.sqrt(x_diff**2 + y_diff**2 + yaw_diff**2)
    return distance

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
            order to calculate a cost. Updates self.__cost

        @return: nothing

        """
        self.__cost = find_distance(self.expected_pose, self.actual_pose)

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

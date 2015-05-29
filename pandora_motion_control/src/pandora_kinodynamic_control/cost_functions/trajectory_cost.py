from cost_graph.cost_node import CostNode
from geometry_msgs.msg import Pose

class TrajectoryCost(CostNode):

    """@brief: Implementation of CostNode for calculate a cost from trajectory
       dissimilarity errors"""

    def __init__(self):
        """@brief: Initiates a TrajectoryCost object"""
        super(TrajectoryCost, self).__init__()
        self.time_granularity = 5
        self.expected_trajectory = list()
        self.actual_trajectory = list()

    def update_cost(self):
        """@brief: Uses expected and actual trajectory (list of poses) in order
        to calculate a cost based on their dissimilarity

        Uses a specific (which?) curve similarity algorithm.
        Updates self.__cost.

        @return: nothing

        """
        # TODO calculate curve similarity
        # self.__cost = ..something

    def calculate_expected_trajectory(self, pose, twist, duration):
        """@brief: Calculates expected trajectory and extends
        self.expected_trajectory with it.

        Trajectory is a curve with resolution defined by self.time_granularity
        according to the twist movement command and the duration which
        the command is to be followed.
        Every expected_trajectory must be a discrete arc of a circle.

        @param pose: Pose, vehicle's initial pose at the time of movement command
        @param twist: Twist, vehicle's movement command (linear & angular vels)
        @param duration: double, how much time will the twist be followed
        @return: nothing

        """
        # TODO calculate trajectory

    def append_actual_pose(self, pose):
        """@brief: Appends an actual pose to the self.actual_trajectory
        discrete curve

        Should be called in respect to the time_granularity set in the
        TrajectoryCost object.

        @param pose: Pose, vehicle's actual pose the moment that granularity
        period has ended
        @return: nothing

        """
        self.actual_trajectory.append(pose)

    def set_actual_trajectory(self, trajectory):
        """@brief: Setter for the actual trajectory of the robot
        among the time instance that the movement command was issued
        and the time instance the movement command has finished
        (after a given duration of time)

        To be called in case there is a mechanism that tracks the vehicle's
        trajectory.

        @param trajectory: list, of Pose
        @return: nothing

        """
        self.actual_trajectory = trajectory

    def get_actual_trajectory(self):
        """@brief: Getter for the actual trajectory done

        @return: list of Pose, the actual trajectory

        """
        return self.actual_trajectory

    def extend_expected_trajectory(self, trajectory):
        """@brief: Extends self.expected_trajectory with  the expected
        trajectory of the robot as calculated by the local planner

        @param trajectory: list, of Pose the expected trajectory
        @return: nothing

        """
        self.expected_trajectory.extend(trajectory)

    def get_expected_trajectory(self):
        """@brief: Getter for the expected trajectory calculated

        @return: list of Pose, the expected trajectory

        """
        return self.expected_trajectory

    def clear_trajectories(self):
        """@brief: Reset actual and expected trajectories to empty lists

        @return: nothing

        """
        self.actual_trajectory = list()
        self.expected_trajectory = list()

    def set_time_granularity(self, time_grans):
        """@brief: Setter for granularity of a curve according to time

        Used when calculating expected trajectory.
        Implied when appending poses to actual trajectory.

        @param time_grans: int, a standard discretization of time which
        defines the resolution of a curve in respect with the time
        @return: nothing

        """
        self.time_granularity = time_grans

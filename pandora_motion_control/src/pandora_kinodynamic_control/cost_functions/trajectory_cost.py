from cost_graph.cost_node import CostNode
from geometry_msgs.msg import Pose

class TrajectoryCost(CostNode):

    """Implementation of CostNode for calculate a cost from trajectory
       dissimilarity errors"""

    def __init__(self):
        """Initiates a TrajectoryCost object"""
        super(TrajectoryCost, self).__init__()
        self.time_granularity = 0.01
        self.expected_trajectory = list()
        self.actual_trajectory = list()

    def update_cost(self):
        """Uses expected and actual trajectory (list of poses) in order to
        calculate a cost based on their dissimilarity. Uses a specific (which?)
        curve similarity algorithm. Updates self.__cost. Will reset
        expected and actual trajectories to empty lists
        @return: nothing

        """
        # TODO calculate curve similarity
        # self.__cost = ..something
        self.expected_trajectory = list()
        self.actual_trajectory = list()

    def set_time_granularity(self, time_grans):
        """Setter for granularity of a curve according to time. Used when
        calculating expected trajectory. Implied when appending poses to
        actual trajectory

        @param time_grans double, a standard duration of time which defines the
        resolution of a curve in respect with the time
        @return: nothing

        """
        self.time_granularity = time_grans

    def calculate_expected_trajectory(self, pose, twist, duration):
        """Calculates self.expected_trajectory curve with a resolution defined
        by self.time_granularity according to the twist movement command and the
        duration which the command is to be followed. Every expected_trajectory
        must be a discrete arc of a circle.

        @param pose Pose, vehicle's initial pose at the time of movement command
        @param twist Twist, vehicle's movement command (linear & angular vels)
        @param duration double, how much time will the twist be followed
        @return: list, of Pose the expected trajectory calculated

        """
        # TODO calculate trajectory
        return self.expected_trajectory

    def append_actual_pose(self, pose):
        """Appends an actual pose to the self.actual_trajectory discrete curve.
        Should be called in respect to the time_granularity set in the
        TrajectoryCost object

        @param pose Pose, vehicle's actual pose the moment that granularity
        period has ended
        @return: list, of Pose the actual trajectory calculated so far

        """
        self.actual_trajectory.append(pose)

    def set_actual_trajectory(self, trajectory):
        """Setter for the actual trajectory of the robot among the time instance
        that the movement command was issued and the time instance the movement
        command has finished (after a given duration of time).
        To be called in case there is a mechanism that tracks the vehicle's
        trajectory

        @param trajectory list, of Pose
        @return: nothing

        """
        self.actual_trajectory = trajectory

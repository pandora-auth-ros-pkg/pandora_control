from cost_functions.goal_cost import GoalCost
from cost_functions.trajectory_cost import TrajectoryCost
from cost_functions.cost_graph.fuse_cost_node import FuseCostNode
from cost_functions.linear_fusion import LinearFusion

class MotionReward(object):

    """@brief: It is responsible for using the environment to calculate
    the reward for execution of a task of an agent

    It uses cost functions which are combined in a directed acyclic graph
    in order to produce a unified cost that an action caused.
    It is also delegated with handling information from the environment.

    """

    def __init__(self):
        """@brief: Initialization of a MotionReward object"""
        self.goal_cost_node = GoalCost()
        self.trajectory_cost_node = TrajectoryCost()
        self.cost_nodes = [self.goal_cost_node, self.trajectory_cost_node]
        self.strategy = LinearFusion()
        self.fuse_cost_node = FuseCostNode(self.cost_nodes, self.strategy)

    def get_reward_from_pose(self, actual_pose):
        """@brief: Returns and calculates the reward associated with a certain
        action taken by the agent who controls the kinodynamic controller

        @param actual_pose: Pose, new information from the environment
        considering the latest action taken
        @return: double, the reward of the action

        """
        self.update_actual_info(actual_pose)
        self.goal_cost_node.set_actual_pose(actual_pose)

        return self.__get_reward()

    def get_reward_from_trajectory(self, actual_trajectory):
        """@brief: Returns and calculates the reward associated with a certain
        action taken by the agent who controls the kinodynamic controller

        Need vehicle's actual trajectory as given from the corresponding topic.

        @param actual_trajectory: list of Pose, actual trajectory of vehicle
        between two time moments of consecutive states
        @return: double, the reward of an action

        """
        self.trajectory_cost_node.set_actual_trajectory(actual_trajectory)
        if len(actual_trajectory) <= 1:
            print "[MotionReward] ERROR: actual_trajectory has length <= 1"
        self.goal_cost_node.set_actual_pose(actual_trajectory[-1])

        return self.__get_reward()

    def __get_reward(self):
        """@brief: Returns reward according to the unified cost return by the
        cost functions

        @return: double, the reward of an action

        """
        self.goal_cost_node.update_cost()
        self.trajectory_cost_node.update_cost()
        self.fuse_cost_node.update_cost()

        cost = self.fuse_cost_node.get_cost()
        reward = 1 / cost  # TODO how should reward be calculated from cost?
        return reward

    def init_info_from_action(self, actual_pose, twist, duration, clear=False):
        """@brief: Initialize cost functions with latest actual pose and
        velocity command

        @param actual_pose: Pose, vehicle's pose at the latest state
        @param twist: Twist, velocity command from local planner
        @param duration: double, how much time will the twist be followed
        @return: nothing

        """
        if clear:
            self.trajectory_cost_node.clear_trajectories()
        self.trajectory_cost_node.calculate_expected_trajectory(actual_pose,
                                                                twist, duration)
        expected_trajectory = self.trajectory_cost_node.get_expected_trajectory()
        if len(expected_trajectory) <= 1:
            print "[MotionReward] ERROR: expected_trajectory has length <= 1"
        goal_pose = expected_trajectory[-1]
        self.goal_cost_node.set_goal_pose(goal_pose)

    def init_info_from_expected(self, expected_trajectory, clear=False):
        """@brief: Initialize cost functions with expected trajectory given from
        local planner

        @param expected_trajectory: list of Pose, the expected trajectory
        @return: nothing

        """
        if clear:
            self.trajectory_cost_node.clear_trajectories()
        self.trajectory_cost_node.extend_expected_trajectory(expected_trajectory)
        if len(expected_trajectory) <= 1:
            print "[MotionReward] ERROR: expected_trajectory has length <= 1"
        goal_pose = expected_trajectory[-1]
        self.goal_cost_node.set_goal_pose(goal_pose)

    def update_actual_info(self, actual_pose):
        """@brief: Update agent's perception of the environment

        @param actual_pose: Pose, new information from the environment
        considering the latest action taken
        @return: nothing

        """
        self.trajectory_cost_node.append_actual_pose(actual_pose)

    def set_params(self, cost_weights, time_granularity):
        """@brief: Configure parameters of cost nodes

        @param cost_weights: list of double, contains weights for the linear
        fusion of the costs
        @param time_granularity: double, resolution of a discrete curve in
        respect to the time
        @return: nothing

        """
        self.strategy.set_weights(cost_weights)
        self.trajectory_cost_node.set_time_granularity(time_granularity)

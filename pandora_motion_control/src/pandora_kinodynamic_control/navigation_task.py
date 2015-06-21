from pybrain.rl.environments.task import Task

from geometry_msgs.msg import Twist

from src.pandora_kinodynamic_control.motion_reward import MotionReward
from src.pandora_kinodynamic_control import utils

class NavigationTask(Task):

    """ @brief: Class that models the navigational task that we want a robotic
        vehicle to do

        Gets information from environment in order to determine vehicle's state.
        Formats actions to messages and send them to vehicle's kinodynamic model
        component.
        Gets the reward for agent's latest action.

    """

    def __init__(self, environment):
        """ @brief: Initialization for a NavigationTask object

        @param environment: An object that handles communication from and to
        this module's system environment
        @note : Although not regiestered as a class variable , NavigationTask holds
        a NavigationEnvironment object . The NavigationEnvironment is used for communication
        between the RL module and the real world.
        @type environment: NavigationEnvironment

        """

        super(NavigationTask, self).__init__(environment)

        self.motion_reward = MotionReward()

        # Navigation Command related 
        self._cmd_vel = Twist()
        self._expected_trajectory = None
        self._actual_trajectory = None

        # Parameters
        self._trajectory_duration = 0.2
        self._time_granularity = 5
        self.discretizing = True

    def set_velocity_command(self, cmd_vel):
        """ @brief: Setter for velocity command from navigation

            Calculating expected trajectory and thus getting action's reward
            depends from it. Should be called before self.getObservation

        @param cmd_vel: navigation's velocity command of center of mass
        @type cmd_vel: Twist
        @return: nothing

        """
        self._cmd_vel = cmd_vel

    def get_observation(self, final=False):
        """ @brief: Calculates environmental info and informs about vehicle's
            state in environment

            Delegates to NavigationEnvironment getSensors

        @override: from Task
        @return: vehicle's state, thus agent's state in MDP

        """
        # Find current pose, return pitch, roll denormalized states
        sensors = self.env.get_sensors()
        # Get current pose as found from environment
        curr_pose = self.env.get_current_pose()
        # Get actual trajectory as resulted from last command
        if final:
            self._actual_trajectory = self.env.find_actual_trajectory()

        # Set latest action's expected trajectory in motion reward object
        if self._expected_trajectory is not None:
            self.motion_reward.init_info_from_expected(self._expected_trajectory)
        # Construct current expected trajectory from current pose, velocity
        # command and trajectory duration in time
        self._expected_trajectory = utils.calculate_expected_trajectory(
            curr_pose, self._cmd_vel,
            self._trajectory_duration, self._time_granularity)

        if not final:
            return list()

        # Make a state vector of (pitch, roll, linear, angular)
        sensors.append(self._cmd_vel.linear.x)
        sensors.append(self._cmd_vel.angular.z)

        if self.sensor_limits:
            sensors = self.normalize(sensors)
        if self.discretizing:
            sensors = self.discretize(sensors)
        return sensors

    def get_reward(self):
        """ @brief: Calculates using MotionReward last action's reward
            and returns it

        @override: from Task
        @return: double, agent's latest action's reward

        """
        return self.motion_reward.get_reward_from_trajectory(self._actual_trajectory)

    def perform_action(self, action):
        """ @brief: Delegates to NavigationEnvironment to create messages and
            order a change in environment according to action argument

        @param action: navigation's velocity command and agent's latest action
        @type action: tuple of Twist and double
        @override: from Task
        @return: nothing

        """
        # Make a final action vector of (velocity command, params)
        final_action = list()
        final_action.append(self._cmd_vel.linear.x)
        final_action.append(self._cmd_vel.angular.z)
        final_action.extend(action)
        # Delegate interaction with environment to NavigationEnvironment
        self.env.perform_action(final_action)

    def discretize(self, sensors):
        """ @brief: Map continuous values to discrete values

        @param sensors: sensor values which define a state in continuous values
        @type sensors: list of doubles
        @return: list of doubles, sensor values but in a discrete set

        """
        return sensors

    def set_params(self, duration, cost_weights, time_granularity):
        """ @brief: Configure parameters for NavigationTask

        @param duration: duration between 2 supposed state changes, length of
        trajectories in time
        @type duration: double
        @param cost_weights: contains weights for the linear fusion of the costs
        @type cost_weights: list of double
        @param time_granularity: resolution of a discrete curve in respect
        to the time
        @type time_granularity: double
        @return: nothing

        """
        self._trajectory_duration = duration
        self._time_granularity = time_granularity
        self.motion_reward.set_params(cost_weights, time_granularity)

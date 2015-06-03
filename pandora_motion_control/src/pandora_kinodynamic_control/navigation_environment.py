from pybrain.rl.environments.environment import Environment

import rospy
import tf
from nav_msgs.msg import Path

from pandora_kinodynamic_control.msg import KinodynamicCommand
from src.pandora_kinodynamic_control.params import *

class NavigationEnvironment(Environment):

    """ @brief: Class for handling interactions with the system environment

        Uses ROS communications to get information useful for understanding
        vehicle's current state, last action's reward and executing current action.

    """

    def __init__(self):
        """ @brief: Initialization for a NavigationEnvironment object"""

        super(NavigationEnvironment, self).__init__()

        self.trajectory_sub = rospy.Subscriber(EXP_TRAJECTORY_TOPIC,
                                               Path, self.expected_trajectory_cb)
        self.command_pub = rospy.Publisher(COMMAND_TOPIC, KinodynamicCommand)
        self.transform_listener = tf.TransformListener()

        self._last_expected_trajectory = None
        self._curr_expected_trajectory = None

        self._last_actual_trajectory = None
        self._curr_pose = None

    def getSensors(self):
        """ @brief: the current visible state of the vehicle in the world

        @return: Details about vehicle's joint state, next expected_trajectory
        and last actual_trajectory

        """
        # format current pose
        # format actual trajectory
        return (self._curr_pose, self._last_actual_trajectory)

    def performAction(self, action):
        """ @brief: perform an action on the world that changes vehicle's state
        and influences vehicle's motion

        @param action: Details about navigation's velocity commands and quality
        params for the kinodynamic model
        @type action: [Twist, params]
        @return: nothing

        """
        command = KinodynamicCommand()
        # TODO format message accordingly
        self.command_pub.publish(command)
        self._last_expected_trajectory = self._curr_expected_trajectory

    def expected_trajectory_cb(self, expected_trajectory):
        """ @brief: Callback of subscriber of navigation's expected trajectory

        @param expected_trajectory: Vehicle's expected movement after
        execution of velocity commands
        @type expected_trajectory: Path
        @return: nothing

        """
        self._curr_expected_trajectory = expected_trajectory

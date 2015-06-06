from pybrain.rl.environments.environment import Environment

import rospy
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import Pose

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

        # self.trajectory_sub = rospy.Subscriber(EXP_TRAJECTORY_TOPIC,
        #                                        Path, self.expected_trajectory_cb)
        self.command_pub = rospy.Publisher(COMMAND_TOPIC, KinodynamicCommand)
        self.transform_listener = tf.TransformListener()

        self._last_moment = None
        self._curr_moment = None

        self._last_actual_trajectory = None
        self.curr_pose = Pose()


    def getSensors(self):
        """ @brief: the current visible state of the vehicle in the world

        @return: Details about vehicle's joint state

        """
        pitch = 0
        roll = 0
        # get and format current pose
        # format actual trajectory
        return [pitch, roll]

    def find_actual_trajectory(self):
        """ @brief: Finds in robot trajectory topic, vehicle's actual trajectory

        Actual trajectory is defined in time by the last time this method was
        called and the next time is called

        @return: list of tuples (x, y, yaw), vehicle's actual trajectory

        """
        pass

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

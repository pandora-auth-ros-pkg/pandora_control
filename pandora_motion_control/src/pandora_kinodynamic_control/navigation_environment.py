from pybrain.rl.environments.environment import Environment

import rospy
import tf
from tf import transformations

from nav_msgs.msg import Path
from geometry_msgs.msg import Point

from pandora_motor_hardware_interface.msg import KinodynamicCommand
from pandora_kinodynamic_control.params import *

class NavigationEnvironment(Environment):

    """ @brief: Class for handling interactions with the system environment

        Uses ROS communications to get information useful for understanding
        vehicle's current state, last action's reward and executing current action.

    """

    def __init__(self):
        """ @brief: Initialization for a NavigationEnvironment object"""

        super(NavigationEnvironment, self).__init__()

        self.trajectory_sub = rospy.Subscriber(ACTUAL_TRAJECTORY_TOPIC,
                                               Path, self.actual_trajectory_cb)
        self.command_pub = rospy.Publisher(COMMAND_TOPIC, KinodynamicCommand)
        self.transform_listener = tf.TransformListener()

        self._last_moment = None
        self._last_index = None
        self._curr_moment = None

        self._actual_trajectory = None
        self._last_actual_trajectory = None

        self.curr_pos = Point()
        self.curr_roll = None
        self.curr_pitch = None
        self.curr_yaw = None

    def get_sensors(self):
        """ @brief: the current visible state of the vehicle in the world

        @return: Details about vehicle's joint state
        @note : Tricky timout var.Must add exception handlers

        """
        try:
            self.transform_listener.waitForTransform(WORLD, BASE_LINK,
                                                     rospy.Time(), rospy.Duration(3))
            trans, rot = self.transform_listener.lookupTransform(WORLD, BASE_LINK,
                                                                 rospy.Time())
        except tf.Exception:
            print "Tf failure! (Not recovery behaviour yet)"

        roll, pitch, yaw = transformations.euler_from_quaternion(rot)
        self.curr_pos = trans
        self.curr_roll = roll
        self.curr_pitch = pitch
        self.curr_yaw = yaw

        return [roll, pitch]

    def get_current_pose(self):
        """ @brief: returns enough info from most recent fetched current pose
            to describe vehicle's 2d pose

        @return: tuple of (x, y, yaw), information of 2d vehicle pose
        @note : changed curr_pos.x ->curr_pos[0] because it was tuple
        """
        return (self.curr_pos[0], self.curr_pos[1], self.curr_yaw)

    def find_actual_trajectory(self):
        """ @brief: Finds in robot trajectory topic, vehicle's actual trajectory

        Actual trajectory is defined in time by the last time this method was
        called and the next time is called

        @return: list of tuples (x, y, yaw), vehicle's actual trajectory
        @note : self._last_moment is not updated ...
                Update every new actual_path
        """

        # Initiallization of time for limiting actual_trajectory from SLAM
        if self._last_moment == None:
            self._last_moment = rospy.Time.now()
            return list()

        # Read trajecotry from SLAM /robot_trajecotry
        actual_path = []
        # The list must be reversed , otherwise it will immedatelly break
        for p in reversed(self._actual_trajectory.poses):

            #Stop if point time stamp < than last_time_stamp
            if p.header.stamp < self._last_moment:
                break

            #Tranformation from geometry_msgs.quaternion to numpy array
            numpy_quaternion = (
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w)
            # transform quaternion to euler angles , using tf library
            (p_roll ,p_pitch ,p_yaw) = transformations.euler_from_quaternion(numpy_quaternion)

            #for every point belonging in last trajectory , save needed information
            info = (p.pose.position.x,p.pose.position.y,p_yaw)
            actual_path.insert(0,info)

        self._last_actual_trajectory = actual_path
        self._last_moment = rospy.Time.now()
        return actual_path

    def perform_action(self, action):
        """ @brief: perform an action on the world that changes vehicle's state
        and influences vehicle's motion

        @param action: Details about navigation's velocity commands and quality
        params for the kinodynamic model
        @type action: [Twist, params]
        @return: nothing

        """
        # Form a new empty kinodynamic command msg
        command = KinodynamicCommand()

        # Fill with information provided by action argument
        command.cmd_vel.linear.x = action[0]
        command.cmd_vel.angular.z = action[1]

        # Scale left/right not used in this version
        command.scale_left = 1
        command.scale_right = 1

        # Fill terrain parameter
        command.terrain_param = action[2]

        # Terrain parameter (= agent's action)
        self.command_pub.publish(command)

    def actual_trajectory_cb(self, actual_trajectory):
        """ @brief: Callback for actual trajectory subscriber

            SLAM publishes robot trajectory from start in a topic

        @param actual_trajectory: a list of actual robot poses
        @type actual_trajectory: Path
        @note: it may need a mutex to assure safety
        BEWARE : in python , callbacks run in a seperate thread by defaul.
        check : http://answers.ros.org/question/110336/python-spin-once-equivalent/ for more information
        @return: nothing

        """
        self._actual_trajectory = actual_trajectory

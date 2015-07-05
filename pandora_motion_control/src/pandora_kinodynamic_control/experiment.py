import rospy
from geometry_msgs.msg import Twist

from pandora_kinodynamic_control.params import *
import scipy

class Experiment(object):

    """ @brief: An experiment matches up a task with an agent and handles
        their interactions"""

    def __init__(self, task, agent):
        """ @brief: Initialization of a MotionReward object"""

        self.task = task
        self.agent = agent

        self.action_done = False

        self.command_sub = rospy.Subscriber(NAVIGATION_TOPIC,
                                            Twist, self.navigation_cb)

        # Keep latest agent's action until update
        self._last_action = scipy.array([1.5])

        # State Change steps
        self.local_step = 0
        self.local_step_size = None

    def navigation_cb(self, cmd_vel):
        """ @brief: Callback that handles velocity commands from navigation

        Logic has been set up so that it can use a SARSA learning algorithm and
        a state change in agent will be initiated when this callback is called.

        @param cmd_vel: velocity command for vehicle's center of mass
        @type cmd_vel: Twist
        @return: nothing

        """
        self.local_step += 1
        self.task.set_velocity_command(cmd_vel)
        final = self.local_step == self.local_step_size
        observation = self.task.get_observation(final)
        # get informed about vehicle's current state
        if final:
            self.local_step = 0

            if self.action_done:
                reward = self.task.get_reward()
                # get informed about last action's reward
                self.agent.giveReward(reward)
                # inform agent about last action's reward
            self.agent.integrateObservation(observation)
            # inform agent about vehicle's current state
            self._last_action = self.agent.getAction()
            # ask agent to decide what the current action will be
            if self.action_done:
                self.agent.learn()
                # ask agent to update its estimations about expected returns
            self.action_done = True

        # perform cmd command in the environment , using agent's last action
        self.task.perform_action(self._last_action)

    def set_params(self, local_step_size):
        """ @brief: Setter method for experiment's parameters

        @param local_step_size: after how many calls to navigational callback we
        will consider that world's state has changed
        @type local_step_size: int
        @return: nothing

        """
        self.local_step_size = local_step_size

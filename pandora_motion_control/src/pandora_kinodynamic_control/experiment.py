import rospy
from geometry_msgs.msg import Twist
from topics import *

class Experiment(object):
    """ @brief: An experiment matches up a task with an agent and handles
        their interactions"""

    def __init__(self, task, agent):
        """ @brief: Initialization of a MotionReward object"""

        self.task = task
        self.agent = agent
        self.stepid = 0

        self.action_done = False

        self.command_subscriber = rospy.Subscriber(NAVIGATION_TOPIC,
                                                   Twist, self.navi_callback)

    def navi_callback(self, cmd_vel):
        """ @brief: Callback that handles velocity commands from navigation

        Logic has been set up so that it can use a SARSA learning algorithm and
        a state change in agent will be initiated when this callback is called.

        @param cmd_vel: velocity command for vehicle's center of mass
        @type cmd_vel: Twist
        @return: nothing

        """
        observation = self.task.getObservation()
        # get informed about vehicle's current state
        if self.action_done:
            reward = self.task.getReward()
            # get informed about last action's reward
            self.agent.giveReward(reward)
            # inform agent about last action's reward
        self.agent.integrateObservation(observation)
        # inform agent about vehicle's current state
        action = self.agent.getAction()
        # ask agent to decide what the current action will be
        if self.action_done:
            self.agent.learn()
            # ask agent to update its estimations about expected returns
        self.task.performAction(action)
        # do the action in the world
        self.action_done = True

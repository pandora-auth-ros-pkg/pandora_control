#!/usr/bin/env python
import rospy
import scipy
from pandora_kinodynamic_control.experiment import Experiment
from pandora_kinodynamic_control.navigation_task import NavigationTask
from pandora_kinodynamic_control.navigation_environment import NavigationEnvironment
from pybrain.rl.learners.valuebased import ActionValueTable
from pybrain.rl.agents import LearningAgent
from pybrain.rl.learners import SARSA
from pandora_kinodynamic_control.params import *

class KinodynamicController(object):
    """ @brief: A class to handle interactions between various components of the RL module"""

    def __init__(self):
        """ @brief: Setting up internal parameters for the RL module"""

        # Navigation Task
        self._environment = NavigationEnvironment()
        self._task = NavigationTask(self._environment)

        # Number of States : (read from params.py)
        self._states = STATES
        self._state_limits = LIMITS

        # Total number of states:
        self._number_of_states = 1
        for i in self._states:
            self._number_of_states *= i

        # Number of actions
        self._actions = ACTION_STATES
        self._action_limits = ACTION_RANGE

        # Action Value Table setup
        self._av_table = ActionValueTable(self._number_of_states, self._actions)
        self._av_table.initialize(0.0)

        # Set up task parameters:
        self._task.set_params(COMMAND_DURATION,
                              FUSION_WEIGHTS,
                              TIME_GRANULARITY,
                              self._state_limits,
                              MAX_REWARD,
                              COST_THRESHOLD)

        # Agent set up
        self._learner = SARSA(alpha,gamma)
        self._agent = LearningAgent(self._av_table, self._learner)

        # Experiment set up
        self._experiment = Experiment(self._task,self._agent)
        self._experiment.set_params(STEP_SIZE)

        print "Successfully Initialization of RL module! (kappa)"


if __name__ == '__main__':
    # Spawn ROS node , create controller and spin!
    rospy.init_node('kinodynamic_controller')
    controller = KinodynamicController()
    rospy.spin()

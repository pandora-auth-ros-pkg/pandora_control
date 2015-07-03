#!/usr/bin/env python

import rospy
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
        self._limits = LIMITS

        self._number_of_states = 1
        for i in self._states:
            self._number_of_states*=i

        print "total_states ="+str(self._number_of_states)
        # Number of actions
        self._actions = ACTIONS
        print "total_actions ="+str(self._actions) 
        self._av_table = ActionValueTable(self._number_of_states, self._actions)
        self._av_table.initialize(0.)

        # Set up task parameters:
        time_granularity = 5
        weights = [1,1]
        command_duration = 0.2
        self._task.set_params(command_duration,weights,
                            time_granularity,self._limits)

        # Agent set up
        self._learner = SARSA(0.5,0.3)
        self._agent = LearningAgent(self._av_table, self._learner)

        # Experiment set up
        self._experiment = Experiment(self._task,self._agent)
        print "Successfully Initialization of RL module! (kappa)"


if __name__ == '__main__':

    rospy.init_node('kinodynamic_controller')
    controller = KinodynamicController()
    rospy.spin()

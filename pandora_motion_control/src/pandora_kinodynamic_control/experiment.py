# Software License Agreement
__version__ = "0.0.1"
__status__ = "Development"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__authors__ = "Zisis Konstantinos"
__maintainer__ = "Zisis Konstantinos"
__email__ = "zisikons@gmail.gr"


import rospy
from geometry_msgs.msg import Twist
from threading import Lock

from pandora_kinodynamic_control.params import *
from pandora_motion_control.srv import StoreAVTable
import scipy

class Experiment(object):

    """ @brief: An experiment matches up a task with an agent and handles
        their interactions"""

    def __init__(self, task, agent):
        """ @brief: Initialization of a MotionReward object"""

        self.task = task
        self.agent = agent
        self.cbLock = Lock()

        self.action_done = False

        self.command_sub = rospy.Subscriber(NAVIGATION_TOPIC,
                                            Twist, self.navigation_cb)

        # Keep latest agent's action until update
        self._last_action = scipy.array([0])
        # State Change steps
        self.local_step = 0
        self.local_step_size = None

        self.save_step_size = SAVE_STEP_SIZE
        self.save_step = 0

    def navigation_cb(self, cmd_vel):
        """ @brief: Callback that handles velocity commands from navigation

        Logic has been set up so that it can use a SARSA learning algorithm and
        a state change in agent will be initiated when this callback is called.

        @param cmd_vel: velocity command for vehicle's center of mass
        @type cmd_vel: Twist
        @return: nothing

        """
        self.cbLock.acquire(False)

        self.local_step += 1
        self.save_step += 1
        self.task.set_velocity_command(cmd_vel)
        final = self.local_step == self.local_step_size
        [observation,ready] = self.task.get_observation(final)

        # If SLAM failed , reduce loacal step so next callback is also final
        if not ready:
            self.local_step-=1

        # get informed about vehicle's current state
        if final and ready:
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

        # Save Action Value Table
        if self.save_step == self.save_step_size:
            self.save_step = 0
            self.save_Table()

        self.cbLock.release()

    def save_Table(self):
        rospy.wait_for_service('store_table')
        try:
            store_caller = rospy.ServiceProxy('store_table', StoreAVTable)
            response = store_caller()
            print "Table Stored !"
        except rospy.ServiceException:
            print "Failed to store Table"

    def set_params(self, local_step_size,):
        """ @brief: Setter method for experiment's parameters

        @param local_step_size: after how many calls to navigational callback we
        will consider that world's state has changed
        @type local_step_size: int
        @return: nothing

        """
        self.local_step_size = local_step_size

from pybrain.rl.environments.task import Task
from motion_reward import MotionReward

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
        @type environment: NavigationEnvironment

        """

        super(NavigationTask, self).__init__()

        self._environment = environment

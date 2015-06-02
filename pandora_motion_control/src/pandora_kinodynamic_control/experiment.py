class Experiment(object):
    """ @brief: An experiment matches up a task with an agent and handles
        their interactions"""

    def __init__(self, task, agent):
        """ @brief: Initialization of a MotionReward object"""

        self.task = task
        self.agent = agent
        self.stepid = 0

    def doInteractions(self, number = 1):
        """ @brief: The default implementation directly maps the methods of the
            agent and the task.

        @return: int, the number of interactions done

        """

        for _ in range(number):
            self._oneInteraction()
        return self.stepid

    def _oneInteraction(self):
        """ @brief: Give the observation to the agent, takes its resulting
            action and returns it to the task.

            Then gives the reward to the agent again and returns it.

        @return: double, the reward of agent's latest action

        """
        self.stepid += 1
        self.agent.integrateObservation(self.task.getObservation())
        self.task.performAction(self.agent.getAction())
        reward = self.task.getReward()
        self.agent.giveReward(reward)
        return reward


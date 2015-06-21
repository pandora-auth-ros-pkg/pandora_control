
# Check again
from src.pandora_kinodynamic_control import experiment
from pybrain.rl.learners.valuebased import ActionValueTable
from pybrain.rl.agents import LearningAgent
from pybrain.rl.learners import SARSA

# Define Action Value Table
# Number of States: 
#             Roll : 
#                range = [-20 , 20] degrees 
#                step_size = 2 degrees
#                count = 21 cases
#             Pitch : 
#                range = [-20 , 20] degrees 
#                step_size = 2 degrees
#                count = 21 cases
#
#             Total = Roll x Pitch = 441 states
#
#
# Number of Actions :
#             Terrain parameter  (a):
#                range = [1,3]
#                step_size = 0.2
#                count = 11 actions

av_table = ActionValueTable(441, 11)
av_table.initialize(0.)

# Learner =  SARSA (can be changed)
#        a = 0.5 (better change to smthing like : A* exp(-t))
#    gamma = 0.3 (small because there is no dependency between 2 states)
learner = SARSA(0.5,0.3)
# learner._setExplorer(EpsilonGreedyExplorer(0.0)
agent = (av_table,learner)
LearningAgent(av_table, learner)


# Define the environment
env = NavigationEnvironment()


# Define the task
task = NavigationTask(env)

# finally, define experiment
experiment = Experiment(task, agent)

if __name__ == '__main__':
  # TODO 
  # Probably crete an experiment object and spin 
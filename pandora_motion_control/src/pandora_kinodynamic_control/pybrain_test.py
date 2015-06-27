#!/usr/bin/env python

from pandora_kinodynamic_control import navigation_task
from pandora_kinodynamic_control import navigation_environment
import rospy

if __name__ == '__main__':

    # New Nav_env
    rospy.init_node('navigation_environment_test')
    test_env = navigation_environment.NavigationEnvironment()
    my_task = navigation_task.NavigationTask(test_env)
    dur = rospy.Duration(3)


    while not rospy.is_shutdown():

        print my_task.get_observation(True)
        rospy.sleep(dur)

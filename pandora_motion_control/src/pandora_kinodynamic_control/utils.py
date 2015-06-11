import math
from tf import transformations

def calculate_expected_trajectory(pose, twist, duration, time_granularity):
    """ @brief: Calculates expected trajectory

        Trajectory is a curve with resolution defined by time_granularity
        according to the twist movement command and the duration which
        the command is to be followed.
        Every expected_trajectory must be a arc of a circle with discrete points

    @param pose: vehicle's initial pose at the time of movement command
    @type pose: tuple of doubles (x, y, yaw)
    @param twist: vehicle's movement command (linear & angular vels)
    @type twist: Twist
    @param duration: how much time will the twist be followed
    @type duration: double
    @param time_granularity: resolution of a discrete curve in respect
    to the time
    @type time_granularity: double

    @return: list of tuples (x, y, yaw), the expected trajectory

    """
    # TODO calculate trajectory
    return None

def find_distance(pose_a, pose_b):
    """ @brief: finds distance between two poses in terms of x, y and yaw

    @param pose_a: first pose
    @type pose_a: tuple of doubles (x, y, yaw)
    @param pose_b: second pose
    @type pose_b: tuple of doubles (x, y, yaw)
    @return: double, euclidean distance of (x, y, yaw)

    """

    x_diff = pose_a[0] - pose_b[0]
    y_diff = pose_a[1] - pose_b[1]
    yaw_diff = pose_a[0] - pose_b[2]

    distance = math.sqrt(x_diff**2 + y_diff**2 + yaw_diff**2)
    return distance

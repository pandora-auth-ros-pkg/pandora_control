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

    quaternion = (
        pose_a.orientation.x,
        pose_a.orientation.y,
        pose_a.orientation.z,
        pose_a.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_a = euler[2]
    quaternion = (
        pose_b.orientation.x,
        pose_b.orientation.y,
        pose_b.orientation.z,
        pose_b.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_b = euler[2]

    x_diff = pose_a.position.x - pose_b.position.x
    y_diff = pose_a.position.y - pose_b.position.y
    yaw_diff = yaw_a - yaw_b

    distance = math.sqrt(x_diff**2 + y_diff**2 + yaw_diff**2)
    return distance

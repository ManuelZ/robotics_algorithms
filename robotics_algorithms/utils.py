# Beware with tf2_geometry_msgs
# I need it to transform a point between frames with "do_transform_point"
# However, PyKDL has not been ported to ROS2, so importing tf2_geometry_msgs
# is not supported yet. 
# from tf2_ros import tf2_geometry_msgs
# 
# https://github.com/ros2/geometry2/pull/360#issuecomment-754651422
#
# So I'm basically going to use the source of that function
import PyKDL
from geometry_msgs.msg import PointStamped
import numpy as np


def transform_to_kdl(t):
    """
    From: https://github.com/ros2/geometry2/blob/93c4c19276f04074e020b1d26923eaf1eca01716/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py#L48
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))


# PointStamped
def do_transform_point(point, transform):
    """
    From:
    https://github.com/ros2/geometry2/blob/93c4c19276f04074e020b1d26923eaf1eca01716/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py#L57
    """
    p = transform_to_kdl(transform) * PyKDL.Vector(point.point.x, point.point.y, point.point.z)
    res = PointStamped()
    res.point.x = p[0]
    res.point.y = p[1]
    res.point.z = p[2]
    res.header = transform.header
    return res


def prob_to_log_odds(x):
    return np.log(x) - np.log(1 - x)


def log_odds_to_prob(x):
    return 1 - (1 / (1 + np.exp(x)))


def linear_mapping_of_values(x, old_min=0, old_max=1, new_min=0, new_max=100):
    """
    https://stackoverflow.com/a/929107/1253729
    """
    old_range = old_max - old_min
    new_range = new_max - new_min
    return ((((x - old_min) * new_range) / old_range) + new_min).astype(int)
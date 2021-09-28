from math import pi
import numpy as np
import rclpy
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import TransformStamped, Point, PointStamped
from rclpy.node import Node
from functools import partial
from nav_msgs.msg import Odometry
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener
from tf2_ros import LookupException, ConnectivityException
from tf2_ros import ExtrapolationException

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


FLOOR_SIZE_X = 1.5 # meters
FLOOR_SIZE_Y = 1.5 # meters
RESOLUTION = 10
PRIOR_P = 0.5

OUT_OF_RANGE = 0.0
TOF_MAX_RANGE = 2.0

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

class EPuckNode(Node):
    def __init__(self):
        super().__init__('epuck_node')
        self.get_logger().info("Epuck node has been started.")


        # Initialize map in odom frame
        self.map = 0.5 * np.ones(int(FLOOR_SIZE_X * FLOOR_SIZE_Y * RESOLUTION))
        for i in range(self.map.size):
            self.map[i] = prob_to_log_odds(PRIOR_P)

        self.__now = self.get_clock().now().to_msg()
        self.__tof_value = OUT_OF_RANGE
        self.__subscriber_tof = self.create_subscription(Range, '/tof', self.__process_tof, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def __process_tof(self, msg):
        
        # To transform from the odom frame to the frame of the TOF sensor
        to_frame   = 'odom'
        from_frame = msg.header.frame_id
        now        = msg.header.stamp
        tof_value  = msg.range
        
        # Create Point message from Range message
        point = PointStamped()
        point.header.frame_id = from_frame
        point.header.stamp    = now
        point.point           = Point(x=tof_value)

        try:

            #for cell in perceptual range of zt:
            for i in range(self.map.size):

                transform = self.tf_buffer.lookup_transform(to_frame, from_frame, now)
                point_transformed = do_transform_point(point, transform)
                print(f"Point in tof frame: {point.point.x:.2f}; Measurement in odom frame: {point_transformed.point.x:.2f}")

                # z = inverse_range_sensor_model()
                # y = trans.transform.translation.y
                # x = trans.transform.translation.x
                # angle = np.atan2(y, x)
                # self.map[cell] = self.map[cell] + prob_to_log_odds(z) - prob_to_log_odds(PRIOR_P)
        
        except (LookupException, ConnectivityException, ExtrapolationException):
            print("EXCEPTION... continuing")
            return


def inverse_range_sensor_model(range, x, y, theta):
    """
    Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
    z is the measurement of the sensor at time t, along with the pose at which the measurement was taken.
    
    "For example, zt might be a sonar scan and a three-dimensional pose variable (x-y coordinates of the robot and heading direction)."
    
    https://www.cs.cmu.edu/~thrun/papers/thrun.occ-journal.pdf
    """
    alpha = 0
    beta = 0


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EPuckNode()
    rclpy.spin(epuck_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    epuck_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
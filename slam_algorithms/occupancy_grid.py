from math import pi
import rclpy
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from functools import partial
from nav_msgs.msg import Odometry

OUT_OF_RANGE = 0.0
INFRARED_MAX_RANGE = 0.04
INFRARED_MIN_RANGE = 0.009
TOF_MAX_RANGE = 1.0
NB_INFRARED_SENSORS = 8
SENSOR_DIST_FROM_CENTER = 0.035


DISTANCE_SENSOR_ANGLE = [
    -15 * pi / 180,   # ps0
    -45 * pi / 180,   # ps1
    -90 * pi / 180,   # ps2
    -150 * pi / 180,  # ps3
    150 * pi / 180,   # ps4
    90 * pi / 180,    # ps5
    45 * pi / 180,    # ps6
    15 * pi / 180,    # ps7
]

class EPuckNode(Node):
    def __init__(self):
        super().__init__('epuck_node')
        self.get_logger().info("Epuck node has been started.")

        # Intialize distance sensors for LaserScan topic
        self.__subscriber_dist_sensors = {}
        self.__distances = {}
        self.__tof_value = OUT_OF_RANGE
        for i in range(NB_INFRARED_SENSORS):
            self.__distances['ps{}'.format(i)] = OUT_OF_RANGE

        for i in range(NB_INFRARED_SENSORS):
            self.__subscriber_dist_sensors['ps{}'.format(i)] = \
                self.create_subscription(Range,
                                         '/ps{}'.format(i),
                                         partial(self.__on_distance_sensor_message, i),
                                         1)

        self.__subscriber_tof = self.create_subscription(Range, '/tof', self.__process_tof, 1)

        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)

        self.__now = self.get_clock().now().to_msg()

        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.__now
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser_scanner'
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.033

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(laser_transform)

        # Main loop self.get_clock
        # self.create_timer(50 / 1000, self.__publish_laserscan_data)
        self.__subscriber_tof = self.create_subscription(Odometry, '/odom', self.__publish_laserscan_data, 1)

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
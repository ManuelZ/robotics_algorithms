from math import pi
import numpy as np
import rclpy
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import TransformStamped, Point, PointStamped, PoseStamped, Pose
from rclpy.node import Node
from functools import partial
from nav_msgs.msg import Odometry
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener
from tf2_ros import LookupException, ConnectivityException
from tf2_ros import ExtrapolationException
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

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


FLOOR_SIZE_X = 3 # meters
FLOOR_SIZE_Y = 3 # meters
RESOLUTION = 100
PRIOR_P = 0.5

WORLD_ORIGIN_X = - FLOOR_SIZE_X / 2.0
WORLD_ORIGIN_Y = - FLOOR_SIZE_Y / 2.0

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

        self.__subscriber_tof = self.create_subscription(Range, '/tof', self.__process_tof, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile    = QoSProfile(
                depth      = 1,
                durability = DurabilityPolicy.TRANSIENT_LOCAL,
                history    = HistoryPolicy.KEEP_LAST,
            )
        )

        # A null static transform to map the 'odom' frame to the 'map' frame
        self.tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)

        self.__generate_map()
        self.pub_map()
        self.create_timer(1, self.pub_map)


    def __generate_map(self):
        """
        Initialize map in odom frame. The map will have an odd size in the x and
        y directions.
        """

        map_size_x = int(FLOOR_SIZE_X * RESOLUTION)
        self.map_size_x = map_size_x if (map_size_x % 2) == 0 else map_size_x + 1
        
        map_size_y = int(FLOOR_SIZE_Y * RESOLUTION)
        self.map_size_y = map_size_y if (map_size_y % 2) == 0 else map_size_y + 1

        self.map = -1 * np.ones( (self.map_size_x, self.map_size_y), dtype=np.int8)

        self.get_logger().info(f"Map size: {self.map.shape}")
    
        #for ix,iy in np.ndindex(self.map.shape):
        #    self.map[ix, iy] = prob_to_log_odds(PRIOR_P)


    def __odom_coords_to_2d_array(self, x, y):
        """
        Transform a coord in the odom frame to coords in a 2D array.
        
        E.g.
        resolution  = 100
        map_size_ x = 101
        map_size_ y = 101
        (0.8, 1.1) -> 0.8 * 100 + 50, 1.1*100 + 50
                   -> (130, 160)
        
        """
        
        x = int(x * RESOLUTION) + self.map_size_x // 2
        y = int(y * RESOLUTION) + self.map_size_y // 2
        return (x, y)


    def __get_perceptual_range(self, origin, target):
        """
        Get the grid cells that belong to the
        line between the origin and target. The returned points' coordinates are int-indexes 
        of the map 2D array.
        
        Based on the Bresenham's line algorithm:
        https://en.wikipedia.org/wiki/Bresenham's_line_algorithm#All_cases
        """
        
        x0, y0 = self.__odom_coords_to_2d_array(origin.point.x, origin.point.y)
        x1, y1 = self.__odom_coords_to_2d_array(target.point.x, target.point.y)
        
        dx =  np.abs(x1 - x0)
        sx = 1 if (x0 < x1) else -1
        
        dy = -np.abs(y1 - y0)
        sy = 1 if (y0 < y1) else -1

        err = dx + dy

        perceptual_range = []
        while True:
            if (np.abs(x0) >= FLOOR_SIZE_X*RESOLUTION) or (np.abs(y0) >= FLOOR_SIZE_Y*RESOLUTION):
                print(f"Skipping point due to map overflow: ({x0}, {y0})")
                break
            perceptual_range.append((x0, y0))
            if (x0 == x1 and y0 == y1):
                break

            e2 = 2 * err

            if (e2 >= dy):
                err += dy
                x0 += sx
            
            if (e2 <= dx):
                err += dx
                y0 += sy
        
        return perceptual_range


    def __process_tof(self, msg):
        
        # To transform from the odom frame to the frame of the TOF sensor
        to_frame   = 'odom'
        from_frame = msg.header.frame_id
        now        = msg.header.stamp
        
        # Create Point message from Range message
        laser_point = PointStamped(point=Point(x=msg.range))
        laser_point.header.frame_id = from_frame
        laser_point.header.stamp    = now

        # Create Point message representing the base of the TOF laser
        laser_base_point = PointStamped(point=Point())
        laser_base_point.header.frame_id = from_frame
        laser_base_point.header.stamp    = now

        try:
            tof_transform = self.tf_buffer.lookup_transform(to_frame, from_frame, now)
                       
            laser_point_in_odom_frame      = do_transform_point(laser_point, tof_transform)
            laser_base_point_in_odom_frame = do_transform_point(laser_base_point, tof_transform)

            # print("Point in tof frame: ({:.2f}, {:.2f}); Point in odom frame: ({:.2f}, {:.2f})".format(
            #     laser_point.point.x,
            #     laser_point.point.y,
            #     laser_point_in_odom_frame.point.x,
            #     laser_point_in_odom_frame.point.y
            # ))

            perceptual_range = self.__get_perceptual_range(
                laser_base_point_in_odom_frame, 
                laser_point_in_odom_frame
            )
        
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        # Mark as empty all cells in the laser range
        for ix,iy in perceptual_range:
            self.mark_map(ix, iy, 0)
            # z = inverse_range_sensor_model()
            # self.map[cell] = self.map[cell] + prob_to_log_odds(z) - prob_to_log_odds(PRIOR_P)

        # Mark as occupied the point returned by the laser        
        lp_in_array_x, lp_in_array_y = self.__odom_coords_to_2d_array(
            laser_point_in_odom_frame.point.x,
            laser_point_in_odom_frame.point.y
        )

        self.mark_map(lp_in_array_x, lp_in_array_y, 100)


    def mark_map(self, ix, iy, value):
        try:
            self.map[ix,iy] = value
        except Exception as e:
            print(f"Problem when marking map: {e}")
    
    def pub_map(self):
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 1 / RESOLUTION
        msg.info.width  = self.map_size_x
        msg.info.height = self.map_size_y
        msg.info.origin.position.x = WORLD_ORIGIN_X
        msg.info.origin.position.y = WORLD_ORIGIN_Y
        msg.data = self.map.T.reshape(self.map.size, order='C').tolist() # row-major order
        
        self.map_publisher.publish(msg)


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
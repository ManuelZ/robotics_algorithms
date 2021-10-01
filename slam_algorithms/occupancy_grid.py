from math import log, pi
import scipy.stats
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
from slam_algorithms.utils import do_transform_point, prob_to_log_odds, log_odds_to_prob, linear_mapping_of_values


FLOOR_SIZE_X = 3 # meters
FLOOR_SIZE_Y = 3 # meters
RESOLUTION = 100

PRIOR_PROB = 0.5
OCC_PROB   = 0.8
FREE_PROB  = 0.2

WORLD_ORIGIN_X = - FLOOR_SIZE_X / 2.0
WORLD_ORIGIN_Y = - FLOOR_SIZE_Y / 2.0

OUT_OF_RANGE = 0.0
TOF_MAX_RANGE = 2.0

USE_PROBABILITIES = True


class EPuckNode(Node):
    def __init__(self):
        super().__init__('epuck_node')
        self.get_logger().info("Epuck node has been started.")

        self.create_subscription(Range, '/tof', self.__process_tof, 1)

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

        self.map = self.__generate_map(logodds=USE_PROBABILITIES)
        self.pub_map(convert_logodds_to_prob=True)
        self.create_timer(1, partial(self.pub_map, convert_logodds_to_prob=USE_PROBABILITIES), clock=self.get_clock())


    def __generate_map(self, logodds=False):
        """
        Initialize map in odom frame. The map will have an odd size in the x and
        y directions.

        Arguments
        logodds: Whether to initialize the map with logodds or not
        """

        map_size_x = int(FLOOR_SIZE_X * RESOLUTION)
        self.map_size_x = map_size_x if (map_size_x % 2) == 0 else map_size_x + 1
        
        map_size_y = int(FLOOR_SIZE_Y * RESOLUTION)
        self.map_size_y = map_size_y if (map_size_y % 2) == 0 else map_size_y + 1

        map = -1 * np.ones( (self.map_size_x, self.map_size_y))
        self.get_logger().info(f"Map size: {map.shape}")
        
        if logodds:
            for ix,iy in np.ndindex(map.shape):
                map[ix, iy] = prob_to_log_odds(PRIOR_PROB)

        return map
    

    def mark_map(self, ix, iy, value):
        try:
            self.map[ix,iy] = value
        except Exception as e:
            print(f"Problem when marking map: {e}")
    

    def get_map_val(self, ix, iy):
        try:
            return self.map[ix, iy]
        except Exception as e:
            print(f"Problem when querying map: {e}")


    def pub_map(self, convert_logodds_to_prob=False):
        """
        Publish to the an OccupancyGrid to the /map topic.
        """
        
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 1 / RESOLUTION
        msg.info.width  = self.map_size_x
        msg.info.height = self.map_size_y
        msg.info.origin.position.x = WORLD_ORIGIN_X
        msg.info.origin.position.y = WORLD_ORIGIN_Y
        
        map = np.copy(self.map)
        if convert_logodds_to_prob:
            map = log_odds_to_prob(map)
            map = linear_mapping_of_values(map)

        msg.data = map.astype(int).T.reshape(map.size, order='C').tolist() # row-major order
        self.map_publisher.publish(msg)


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
        
        ix = int(x * RESOLUTION) + self.map_size_x // 2
        iy = int(y * RESOLUTION) + self.map_size_y // 2
        return (ix, iy)
        

    def __get_perceptual_range(self, origin, target):
        """
        Get the grid cells that belong to the
        line between the origin and target. The returned points' coordinates are int-indexes 
        of the map 2D array.
        
        Based on the Bresenham's line algorithm, pag 13:
        http://members.chello.at/~easyfilter/Bresenham.pdf 
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
            perceptual_range.append((x0, y0))
            e2 = 2 * err

            if (e2 >= dy):
                if (x0 == x1): break
                err += dy
                x0 += sx
            
            if (e2 <= dx):
                if (y0 == y1): break
                err += dx
                y0 += sy
        
        return perceptual_range


    def __process_tof(self, msg):
        
        # Transform measurements in the TOF frame to the odom frame
        to_frame   = 'odom'
        from_frame = msg.header.frame_id
        now        = msg.header.stamp
        
        # Create Point message from Range message
        laser_point = PointStamped(point=Point(x=msg.range))
        laser_point.header.frame_id = from_frame
        laser_point.header.stamp = now

        # Create Point message representing the base of the TOF laser
        laser_base_point = PointStamped(point=Point())
        laser_base_point.header.frame_id = from_frame
        laser_base_point.header.stamp = now

        try:
            tof_transform = self.tf_buffer.lookup_transform(to_frame, from_frame, now)
            lp_in_odom_frame      = do_transform_point(laser_point, tof_transform)
            lp_base_in_odom_frame = do_transform_point(laser_base_point, tof_transform)

        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        perceptual_range = self.__get_perceptual_range(
            lp_base_in_odom_frame, lp_in_odom_frame
        )

        for i, (ix,iy) in enumerate(perceptual_range):
            p = self.inverse_range_sensor_model(i, len(perceptual_range))
            l_prev = self.get_map_val(ix, iy)
            if l_prev is None: continue
            l = l_prev + prob_to_log_odds(p) - prob_to_log_odds(PRIOR_PROB)    
            self.mark_map(ix, iy, l)


    def inverse_range_sensor_model(self, i, len_perceptual_range):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        This is a naive implementation.
        """
        return OCC_PROB if i == (len_perceptual_range - 1) else FREE_PROB 


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
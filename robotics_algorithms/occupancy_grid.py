# External imports
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from tf2_ros.buffer import Buffer
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros import LookupException
from tf2_ros import ConnectivityException
from tf2_ros import ExtrapolationException
from sensor_msgs.msg import Range
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from rclpy.executors import ExternalShutdownException
from builtin_interfaces.msg import Time
from tf2_geometry_msgs import do_transform_point

# Local imports
from robotics_algorithms.utils import prob_to_log_odds
from robotics_algorithms.utils import log_odds_to_prob
from robotics_algorithms.utils import linear_mapping_of_values


# double the actual world size to let it drift
FLOOR_SIZE_X = 3 # meters, 
FLOOR_SIZE_Y = 3 # meters
RESOLUTION = 0.01 # meters per cell

WORLD_ORIGIN_X = -FLOOR_SIZE_X / 2.0
WORLD_ORIGIN_Y = -FLOOR_SIZE_Y / 2.0

MAP_SIZE_X = int(FLOOR_SIZE_X / RESOLUTION)
MAP_SIZE_Y = int(FLOOR_SIZE_Y / RESOLUTION)

PRIOR_PROB = 0.5
OCC_PROB   = 0.8
FREE_PROB  = 0.2


class EPuckNode(Node):
    def __init__(self):
        super().__init__('epuck_node')
        self.get_logger().info("Epuck node has been started.")

        # Once the listener is created, it starts receiving tf2 transformations
        # over the wire, and buffers them for up to 10 seconds
        # https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Range, '/tof', self.__process_tof, 1)

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
        # For details on the coordinate frames, see
        # https://www.ros.org/reps/rep-0105.html
        # https://www.ros.org/reps/rep-0120.html
        self.tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)

        self.occupancy_grid_msg = OccupancyGrid()
        self.occupancy_grid_msg.header.frame_id = 'map'       
        self.occupancy_grid_msg.info = MapMetaData(
            width      = MAP_SIZE_X,
            height     = MAP_SIZE_Y,
            resolution = RESOLUTION,
            origin     = Pose(
                position = Point(x = WORLD_ORIGIN_X, y = WORLD_ORIGIN_Y)
            )
        ) 

        self.map = self.__generate_map()
        self.pub_map()
        self.create_timer(1, self.pub_map, clock=self.get_clock())


    def __generate_map(self):
        """
        Initialize map in the odom frame.
        """
        self.get_logger().info(f"Generating map of size: {(MAP_SIZE_X, MAP_SIZE_Y)}")
        map = prob_to_log_odds(PRIOR_PROB) * np.ones( (MAP_SIZE_X, MAP_SIZE_Y))
        return map
    

    def mark_map(self, ix, iy, value):
        """
        Update a map's cell value.
        """
        
        try:
            self.map[ix,iy] = value
        except Exception as e:
            print(f"Problem when marking map: {e}")
    

    def get_map_val(self, ix, iy):
        """
        Get a map's cell value.
        """
        try:
            return self.map[ix, iy]
        except Exception as e:
            print(f"Problem when querying map: {e}")


    def pub_map(self):
        """
        Publish an OccupancyGrid to the /map topic.
        """
        
        map = log_odds_to_prob(self.map)
        map = linear_mapping_of_values(map) # To visualize in Rviz

        self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_grid_msg.data = map.astype(int).T.reshape(map.size, order='C').tolist() # row-major order
        self.map_publisher.publish(self.occupancy_grid_msg)


    def __odom_coords_to_2d_array(self, x, y):
        """
        Transform a coord in the odom frame to coords in a 2D array.
        
        E.g.
        resolution  = 0.1
        map_size_ x = 101
        map_size_ y = 101
        (0.8, 1.1) -> 0.8 / 0.1 + 50, 1.1*100 + 50
                   -> (130, 160)
        
        """
        
        ix = int(x / RESOLUTION) + MAP_SIZE_X // 2
        iy = int(y / RESOLUTION) + MAP_SIZE_Y // 2
        
        return (ix, iy)
        

    def __get_perceptual_range(self, origin, target):
        """
        Get the grid cells that belong to the line between the origin and target.
        The returned points' coordinates are int-indexes of the map 2D array.
        
        Based on the Bresenham's line algorithm, pag 13:
        http://members.chello.at/~easyfilter/Bresenham.pdf 
        """
        
        x0, y0 = self.__odom_coords_to_2d_array(origin.point.x, origin.point.y)
        x1, y1 = self.__odom_coords_to_2d_array(target.point.x, target.point.y)
        
        dx = np.abs(x1 - x0)
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
        """
        Process a new TOF laser message and update map.
        """
       
        # Transform measurements in the TOF frame to the odom frame
        from_frame = msg.header.frame_id
        to_frame   = 'odom'
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
            tof_transform = self.tf_buffer.lookup_transform(to_frame, from_frame, Time(sec=0, nanosec=0))
            lp_in_odom_frame = do_transform_point(laser_point, tof_transform)
            lp_base_in_odom_frame = do_transform_point(laser_base_point, tof_transform)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(f"Problem when processing TOF message: ", e)
            return

        perceptual_range = self.__get_perceptual_range(
            lp_base_in_odom_frame, lp_in_odom_frame
        )

        for i, (ix,iy) in enumerate(perceptual_range):
            p = self.inverse_range_sensor_model(i, len(perceptual_range))
            l_prev = self.get_map_val(ix, iy)
            if l_prev is None: continue
            l = l_prev + prob_to_log_odds(p) - prob_to_log_odds(PRIOR_PROB)
            self.mark_map(ix, iy, value=l)


    def inverse_range_sensor_model(self, i, len_perceptual_range):
        """
        Specifies the probability of occupancy of the grid cell m_(x,y) conditioned on the measurement z.
        This is a naive implementation.
        """
        if i == (len_perceptual_range - 1):
            return OCC_PROB
        else:
            return FREE_PROB 


def main(args=None):
    try:
        rclpy.init(args=args)
        epuck_controller = EPuckNode()
        rclpy.spin(epuck_controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        rclpy.try_shutdown()
        epuck_controller.destroy_node()


if __name__ == '__main__':
    main()
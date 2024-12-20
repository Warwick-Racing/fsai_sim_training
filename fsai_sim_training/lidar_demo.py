import rclpy
from rclpy.node import Node
import sensor_msgs.msg
from sensor_msgs_py.point_cloud2 import read_points

import itertools
import numpy as np

class LidarDemo(Node):
    def __init__(self):
        super().__init__('lidar_demo')

        self.__sub = self.create_subscription(sensor_msgs.msg.PointCloud2, "/fsaivehicle/xt32/point_cloud", 
                                              self.__cones_callback, 1)
    
    def __cones_callback(self, msg):

        self.get_logger().info("Callback triggered")
        count = 0

        validPoints = ( (c,i) for c, i in enumerate(read_points(msg)) if not np.isinf(i[0]) )
        for c, i in itertools.islice(validPoints, 10):
            self.get_logger().info(f"Point {c}: {i}")
           

def main(args=None):
    rclpy.init(args=args)
    path_plan = LidarDemo()
    rclpy.spin(path_plan)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_plan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
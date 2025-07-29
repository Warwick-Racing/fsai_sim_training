import rclpy
import rclpy.time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from tf2_ros.buffer import Buffer

import sensor_msgs_py.point_cloud2 as pc2 
import PyKDL

def transform_to_kdl(t):
    return PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ),
        PyKDL.Vector(
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ),
    )

def do_transform_cloud(cloud, transform):
    t_kdl = transform_to_kdl(transform)
    for p_in in cloud:
        p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
        yield p_out[0], p_out[1], p_out[2]

class LidarTfDemo(Node):
    def __init__(self):
        super().__init__('lidar_tf_demo')

        self.__sub = self.create_subscription(
            PointCloud2,
            "/fsaivehicle/xt32/point_cloud",
            self.pointcloud_callback,
            10
        )

        self.__pub = self.create_publisher(
            PointCloud2,
            "/fsaivehicle/xt32/point_cloud_out",
            10
        )

        self.__tfBuffer = Buffer()
        self.__listener = TransformListener( self.__tfBuffer, self )

    def pointcloud_callback(self, msg):
        self.get_logger().info("Got cloud")

        try:
            # Lookup transform from point cloud frame to base_link
            transform = self.__tfBuffer.lookup_transform(
                'base_link',  # target frame
                msg.header.frame_id,  # source frame
                rclpy.time.Time() # msg header.stamp?
            )
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return
           
        # Read points from the incoming PointCloud2
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convert transform to matrix
        transformed = do_transform_cloud( points, transform )

        # Create a new PointCloud2 message
        header = msg.header
        header.frame_id = 'base_link'

        # Create a new PointCloud2 message
        transformed_cloud = pc2.create_cloud_xyz32(
            header,
            list(transformed)
        )

        # Publish the transformed point cloud
        self.__pub.publish(transformed_cloud)
        self.get_logger().info("Published transformed cloud")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTfDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
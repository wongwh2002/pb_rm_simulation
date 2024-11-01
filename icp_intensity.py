import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class AddIntensityNode(Node):
    def __init__(self):
        super().__init__('add_intensity_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar/pointcloud',
            self.pointcloud_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/livox/lidar/pointcloud_intensity', 10)

    def pointcloud_callback(self, msg):
        # Extract original points and add intensity
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        cloud_with_intensity = [(x, y, z, 1.0) for x, y, z in cloud_points]  # Add a default intensity

        # Define PointField structure
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Create a new PointCloud2 message
        new_msg = pc2.create_cloud(msg.header, fields, cloud_with_intensity)
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AddIntensityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

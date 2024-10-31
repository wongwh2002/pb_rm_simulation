import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pcl  # Install using pip install pcl-py
from pcl import pcl_visualization

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('point_cloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.listener_callback,
            10)
        self.get_logger().info('PointCloudSaver Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info('Received point cloud data.')
        # Convert ROS PointCloud2 message to PCL format here
        pcl_data = pcl.PointCloud_PointXYZRGB()
        # Assume pcl.fromROSMsg exists in the package you're using
        pcl.fromROSMsg(msg, pcl_data)
        pcl.io.savePCDFileASCII("livox_pointcloud.pcd", pcl_data)
        self.get_logger().info("Saved point cloud data to livox_pointcloud.pcd")

def main(args=None):
    rclpy.init(args=args)
    point_cloud_saver = PointCloudSaver()
    rclpy.spin(point_cloud_saver)
    point_cloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

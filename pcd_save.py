import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('point_cloud_saver')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(PointCloud2, "/livox/lidar/pointcloud", self.pointcloud_callback, qos_profile)
        self.get_logger().info('PointCloudSaver Node has been started.')

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data.')
        point_cloud = self.pointcloud2_to_open3d(msg)
        o3d.io.write_point_cloud("livox_pointcloud.pcd", point_cloud)
        self.get_logger().info("Saved point cloud data to livox_pointcloud.pcd")

    def pointcloud2_to_open3d(self, cloud_msg):
        # Convert ROS2 PointCloud2 message to Open3D PointCloud
        points = []
        for i in range(0, len(cloud_msg.data), cloud_msg.point_step):
            x, y, z = struct.unpack_from('fff', cloud_msg.data, offset=i)
            points.append([x, y, z])
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(np.array(points))
        return point_cloud

def main(args=None):
    rclpy.init(args=args)
    point_cloud_saver = PointCloudSaver()
    rclpy.spin(point_cloud_saver)
    point_cloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

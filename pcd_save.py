import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        # Subscribe to the cumulative map topic (replace with actual topic)
        self.create_subscription(PointCloud2, "/Laser_map", self.map_callback, qos_profile)
        self.get_logger().info('MapSaver Node has been started.')

    def map_callback(self, msg):
        self.get_logger().info('Received map data.')
        map_cloud = self.pointcloud2_to_open3d(msg)
        o3d.io.write_point_cloud("fastlio_map.pcd", map_cloud)
        self.get_logger().info("Saved map data to fastlio_map.pcd")

    def pointcloud2_to_open3d(self, cloud_msg):
        points = []
        for i in range(0, len(cloud_msg.data), cloud_msg.point_step):
            x, y, z = struct.unpack_from('fff', cloud_msg.data, offset=i)
            points.append([x, y, z])
        map_cloud = o3d.geometry.PointCloud()
        map_cloud.points = o3d.utility.Vector3dVector(np.array(points))
        return map_cloud

def main(args=None):
    rclpy.init(args=args)
    map_saver = MapSaver()
    rclpy.spin(map_saver)
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

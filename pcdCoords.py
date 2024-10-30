import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

def pointcloud_callback(msg):
    points = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])  # Collecting (x, y, z) data
    
    points = np.array(points)
    # Compute the gradient in the z direction (elevation change)
    dx = np.gradient(points[:, 0])
    dz = np.gradient(points[:, 2])
    
    slope = dz / dx  # Ramp detection using change in z over x
    if np.mean(slope) > threshold:  # Define a threshold for what slope is considered a ramp
        print("Ramp detected!")
    else:
        print("Flat ground detected!")

def listener():
    rospy.init_node("pcd_listener")
    rospy.Subscriber("/pointcloud", PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == "__main__":
    listener()

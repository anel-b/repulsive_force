#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import PoseStamped, WrenchStamped
import open3d as o3d
import numpy as np

class RepulsiveForcePublisher(Node):
    def __init__(self):
        super().__init__('F_repulsion_publisher')
        # Subscriber to get end effector position from robot arm
        self.end_effector_position = np.zeros(3)
        self.subscription_robot_state = self.create_subscription(FrankaRobotState, 'franka_robot_state_broadcaster/robot_state', self.robot_state_callback, 1)

        # Publisher for point cloud data to be visualized
        self.publisher_point_cloud = self.create_publisher(PointCloud2, 'point_cloud_topic', 1)

        # Publisher for repulsive force to be used in cartesian impedance controller
        self.publisher_F_repulsion = self.create_publisher(WrenchStamped, 'F_repulsion_topic', 1)

        # Homogeneous transformation matrix from robot base frame (R) to checkerboard frame (B)
        R_T_RB = np.array([[-1.000,  0.000,  0.000,  0.358],
                           [ 0.000,  1.000,  0.000,  0.030],
                           [ 0.000,  0.000, -1.000,  0.006],
                           [ 0.000,  0.000,  0.000,  1.000]])

        # Homogeneous transformation matrix from checkerboard frame (B) to camera frame (C)
        B_T_BC = np.array([[ 0.5357,  0.5685, -0.6244,  0.5918],
                           [-0.8444,  0.3671, -0.3902,  0.6178],
                           [ 0.0074,  0.7363,  0.6767, -0.9096],
                           [ 0.0000,  0.0000,  0.0000,  1.0000]])

        # Homogeneous transformation matrix for correcting camera orientation and position
        C_T_CC = np.array([[ 1.000,  0.000,  0.000,  0.140],
                           [ 0.000, -1.000,  0.000,  0.040],
                           [ 0.000,  0.000, -1.000, -0.040],
                           [ 0.000,  0.000,  0.000,  1.000]])

        # Homogeneous transformation matrix from robot base frame (R) to camera frame (C)
        self.R_T_RC = R_T_RB @ B_T_BC @ C_T_CC

        # Read point cloud data from .ply file
        self.file = '/home/anyba/franka_ros2_ws/src/repulsive_force/point_cloud_data/pc_workspace.ply'
        self.point_cloud = o3d.io.read_point_cloud(self.file)

        # Timer for publishing repulsive force every 0.03 seconds
        self.timer = self.create_timer(0.03, self.publish_F_repulsion)

    def robot_state_callback(self, msg: FrankaRobotState):
        # Get end effector position
        x = msg.o_t_ee.pose.position.x
        y = msg.o_t_ee.pose.position.y
        z = msg.o_t_ee.pose.position.z
        self.end_effector_position = np.array([x, y, z])

        self.get_logger().info('End effector position: %s' % self.end_effector_position)

    def remove_background(self, point_cloud: o3d.geometry.PointCloud):
        # Crop point cloud to workspace
        min_bound = np.array([-0.4, -0.9, -0.03])
        max_bound = np.array([1.2, 0.9, 1.4])
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        point_cloud = point_cloud.crop(bbox)

        return point_cloud

    def remove_robot_arm(self, point_cloud: o3d.geometry.PointCloud):
        # Split point cloud into smaller workspace and outside workspace
        min_bound = np.array([-0.4, -0.9, 0.01])
        max_bound = np.array([1.2, 0.9, 1.4])
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        indices = bbox.get_point_indices_within_bounding_box(point_cloud.points)
        point_cloud_workspace = point_cloud.select_by_index(indices, invert=False)
        point_cloud_outside = point_cloud.select_by_index(indices, invert=True)

        # Apply DBSCAN clustering
        dbscan = DBSCAN(eps=0.089, min_samples=100)
        dbscan.fit(np.asarray(point_cloud_workspace.points))
        labels = dbscan.labels_

        # Find robot arm label with smallest average distance to origin
        unique_labels = np.unique(labels)
        distances = np.linalg.norm(np.asarray(point_cloud_workspace.points), axis=1)
        avg_distances = np.zeros(len(unique_labels))
        for i, label in enumerate(unique_labels):
            avg_distances[i] = np.mean(distances[labels == label])
        min_avg_distance_index = np.argmin(avg_distances)
        robot_arm_label = unique_labels[min_avg_distance_index]

        # Remove robot arm from workspace
        mask = (labels != robot_arm_label)
        points_data = (np.asarray(point_cloud_workspace.points))[mask]
        colors_data = (np.asarray(point_cloud_workspace.colors))[mask]
        point_cloud_workspace.points = o3d.utility.Vector3dVector(points_data)
        point_cloud_workspace.colors = o3d.utility.Vector3dVector(colors_data)

        return point_cloud_workspace + point_cloud_outside

    def add_obstacle(self, point_cloud: o3d.geometry.PointCloud):
        # Define dimensions of obstacle
        min_bound = np.array([0.45, -0.1, 0.4])
        max_bound = np.array([0.55, 0.1, 0.6])
        step = 0.01
        x = np.arange(min_bound[0], max_bound[0], step)
        y = np.arange(min_bound[1], max_bound[1], step)
        z = np.arange(min_bound[2], max_bound[2], step)
        xx, yy, zz = np.meshgrid(x, y, z)

        # Create point cloud of red obstacle
        points = np.vstack((xx.ravel(), yy.ravel(), zz.ravel())).T
        colors = np.zeros((len(points), 3))
        colors[:, 0] = 1.0
        obstacle = o3d.geometry.PointCloud()
        obstacle.points = o3d.utility.Vector3dVector(points)
        obstacle.colors = o3d.utility.Vector3dVector(colors)

        return point_cloud + obstacle

    def get_point_cloud(self):
        # Point cloud preprocessing
        point_cloud = self.point_cloud.uniform_down_sample(every_k_points=40)
        point_cloud = point_cloud.transform(self.R_T_RC)
        point_cloud = self.remove_background(point_cloud)
        point_cloud = self.remove_robot_arm(point_cloud)
        point_cloud, _ = point_cloud.remove_radius_outlier(nb_points=10, radius=0.05)
        point_cloud = self.add_obstacle(point_cloud)

        return point_cloud

    def publish_point_cloud(self, point_cloud: o3d.geometry.PointCloud):
        # Save XYZ and RGB values into numpy arrays
        points_data = np.asarray(point_cloud.points)
        colors_data = np.asarray(point_cloud.colors)
        point_cloud_data = np.concatenate((points_data.astype(np.float32), colors_data.astype(np.float32)), axis=1)

        # Publish point cloud data as PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = 'panda_link0'
        msg.height = 1
        msg.width = len(point_cloud_data)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1)]
        msg.is_bigendian = False
        msg.point_step = np.dtype(np.float32).itemsize * 6 # 32-bit float takes 8 bytes * 6 floats (XYZRGB values)
        msg.row_step = msg.point_step * len(point_cloud_data)
        msg.data = point_cloud_data.tobytes()
        msg.is_dense = False
        self.publisher_point_cloud.publish(msg)

        self.get_logger().info('Point cloud data published!')

    def get_nearest_point(self, point_cloud: o3d.geometry.PointCloud):
        # Search nearest point in point cloud relative to end effector position with KDTree
        point_cloud_tree = o3d.geometry.KDTreeFlann(point_cloud)        
        [_, idx, dist] = point_cloud_tree.search_knn_vector_3d(self.end_effector_position, 1)
        coordinates = point_cloud.points[idx[0]]
        distance = np.sqrt(dist[0])

        return coordinates, distance

    def publish_F_repulsion(self):
        # Get point cloud data
        point_cloud = self.get_point_cloud()

        # Publish point cloud data
        self.publish_point_cloud(point_cloud)

        # Get nearest point in point cloud relative to end effector position
        coordinates, distance = self.get_nearest_point(point_cloud)

        # Calculate F_repulsion
        safe_distance = 0.2
        gain = 10.0
        if distance == 0:
            distance = 0.001
            coordinates[2] = -0.001
        if distance < safe_distance:
            F_repulsion = gain * np.log(safe_distance / distance) * (self.end_effector_position - coordinates) / distance
        else:
            F_repulsion = np.zeros(3)

        # Limit F_repulsion to 50 N
        if np.linalg.norm(F_repulsion) > 50.0:
            F_repulsion = 50.0 * F_repulsion / np.linalg.norm(F_repulsion)

        # Publish F_repulsion as WrenchStamped message
        msg = WrenchStamped()
        msg.wrench.force.x = F_repulsion[0]
        msg.wrench.force.y = F_repulsion[1]
        msg.wrench.force.z = F_repulsion[2]
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.publisher_F_repulsion.publish(msg)

        self.get_logger().info('F_repulsion: %s' % F_repulsion)

def main(args=None):
    rclpy.init(args=args)
    F_repulsion_publisher = RepulsiveForcePublisher()
    rclpy.spin(F_repulsion_publisher)
    F_repulsion_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

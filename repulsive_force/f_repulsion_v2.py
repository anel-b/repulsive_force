#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import PoseStamped, WrenchStamped
import pyzed.sl as sl
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

        # Initialize ZED camera
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.sdk_verbose = False
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        self.init_params.coordinate_units = sl.UNIT.METER
        self.runtime_parameters = sl.RuntimeParameters()

        # Open ZED camera
        self.err = self.zed.open(self.init_params)
        if self.err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().info('Error {}, exit program'.format(self.err))
            self.zed.close()
            exit(1)

        # Transformation matrices from camera calibration
        base_matrix = np.array([[-1.000,  0.000,  0.000,  0.358],
                                [ 0.000,  1.000,  0.000,  0.030],
                                [ 0.000,  0.000, -1.000,  0.006],
                                [ 0.000,  0.000,  0.000,  1.000]])

        transform_matrix = np.array([[ 0.5357,  0.5685, -0.6244,  0.5918],
                                     [-0.8444,  0.3671, -0.3902,  0.6178],
                                     [ 0.0074,  0.7363,  0.6767, -0.9096],
                                     [ 0.0000,  0.0000,  0.0000,  1.0000]])

        rotation_matrix = np.array([[ 1.000,  0.000,  0.000,  0.140],
                                    [ 0.000, -1.000,  0.000,  0.040],
                                    [ 0.000,  0.000, -1.000, -0.040],
                                    [ 0.000,  0.000,  0.000,  1.000]])

        # Transformation matrix from camera frame to robot base frame
        self.transformation = base_matrix @ transform_matrix @ rotation_matrix

        # Timer for publishing repulsive force every 0.03 seconds
        self.timer = self.create_timer(0.03, self.publish_F_repulsion)

    def robot_state_callback(self, msg: FrankaRobotState):
        # Get end effector position
        x = msg.o_t_ee.pose.position.x
        y = msg.o_t_ee.pose.position.y
        z = msg.o_t_ee.pose.position.z
        self.end_effector_position = np.array([x, y, z])

        self.get_logger().info('End effector position: %s' % self.end_effector_position)

    def extract_point_cloud(self, point_cloud_camera: sl.Mat):
        # Extract point cloud data from ZED camera
        point_cloud_data = point_cloud_camera.get_data()
        points_data = point_cloud_data[:, :, :3].reshape(-1, 3)
        rgba_data = point_cloud_data[:, :, 3].reshape(-1, 1)

        # Convert RGBA data to RGB values
        rgba_data = int(rgba_data)
        r_data = (rgba_data >> 24) & 0xFF
        g_data = (rgba_data >> 16) & 0xFF
        b_data = (rgba_data >> 8) & 0xFF
        colors_data = np.stack((r_data, g_data, b_data), axis=1) / 255.0

        # Save point cloud data as Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points_data)
        point_cloud.colors = o3d.utility.Vector3dVector(colors_data)

        return point_cloud

    def remove_background(self, point_cloud: o3d.geometry.PointCloud):
        # Crop point cloud to workspace
        min_bound = np.array([-0.3, -1.2, -0.03])
        max_bound = np.array([1.2, 1.2, 1.2])
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        point_cloud = point_cloud.crop(bbox)

        return point_cloud

    def remove_robot_arm(self, point_cloud: o3d.geometry.PointCloud):
        # Split point cloud into smaller workspace and outside workspace
        min_bound = np.array([-0.3, -1.2, 0.01])
        max_bound = np.array([1.2, 1.2, 1.2])
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
        min_bound = np.array([0.49, -0.2, 0.1])
        max_bound = np.array([0.5, 0.3, 0.9])
        step = 0.02
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
        # Get point cloud data from ZED camera
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            point_cloud_camera = sl.Mat()
            self.zed.retrieve_measure(point_cloud_camera, sl.MEASURE.XYZRGBA)
            point_cloud = self.extract_point_cloud(point_cloud_camera)
        else:
            self.get_logger().info('Error grabbing point cloud data!')
            exit(1)

        # Point cloud preprocessing
        point_cloud = point_cloud.uniform_down_sample(every_k_points=40)
        point_cloud = point_cloud.transform(self.transformation)
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
        msg.header.frame_id = 'world'
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
        distance = dist[0]

        return coordinates, distance

    def publish_F_repulsion(self):
        # Get point cloud data
        point_cloud = self.get_point_cloud()

        # Publish point cloud data
        self.publish_point_cloud(point_cloud)

        # Get nearest point in point cloud relative to end effector position
        coordinates, distance = self.get_nearest_point(point_cloud)

        # Calculate F_repulsion
        safe_distance = 0.05
        gain = 4.0
        if distance < safe_distance:
            if distance == 0:
                distance = 1e-6
            F_repulsion = gain * ((1.0 / distance - 1.0 / safe_distance) ** 0.6) * np.gradient(self.end_effector_position - coordinates)
        else:
            F_repulsion = np.zeros(3)

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

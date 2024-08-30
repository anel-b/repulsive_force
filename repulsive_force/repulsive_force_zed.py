#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sklearn.preprocessing import MinMaxScaler
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import PoseStamped, WrenchStamped
import pyzed.sl as sl
import open3d as o3d
import numpy as np

class RepulsiveForcePublisher(Node):
    def __init__(self):
        super().__init__('repulsive_force_publisher')
        # Subscriber to get end effector position from robot arm
        self.end_effector_position = np.zeros(3)
        self.subscription_robot_state = self.create_subscription(FrankaRobotState, 'franka_robot_state_broadcaster/robot_state', self.robot_state_callback, 1)

        # Publisher for point cloud data to be visualized
        self.publisher_point_cloud = self.create_publisher(PointCloud2, 'point_cloud_topic', 1)

        # Publisher for repulsive force to be used in cartesian impedance controller
        self.publisher_repulsive_force = self.create_publisher(WrenchStamped, 'repulsive_force_topic', 1)

        # Initialize ZED camera
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.set_from_serial_number(30635524)
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        self.init_params.coordinate_units = sl.UNIT.METER

        # Open ZED camera
        self.err = self.zed.open(self.init_params)
        if self.err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().info('Error %s, exit program' % self.err)
            self.zed.close()
            exit(1)

        # Initialize indices for random sampling of point cloud data by 2.0%
        self.number_of_points = 720 * 1280
        self.sample = int(self.number_of_points * 0.020)
        self.indices = np.random.choice(self.number_of_points, self.sample, replace=False)

        # Homogeneous transformation matrix from checkerboard frame B to robot base frame R
        T_RB = np.array([[-1.000,  0.000,  0.000,  0.358],
                         [ 0.000,  1.000,  0.000,  0.030],
                         [ 0.000,  0.000, -1.000,  0.006],
                         [ 0.000,  0.000,  0.000,  1.000]])

        # Homogeneous transformation matrix from camera frame C to checkerboard frame B
        T_BC = np.array([[ 0.8690, -0.1971,  0.4538, -0.5035],
                         [ 0.4943,  0.3069, -0.8133,  1.0069],
                         [ 0.0210,  0.9311,  0.3642, -0.6867],
                         [ 0.0000,  0.0000,  0.0000,  1.0000]])

        # Homogeneous transformation matrix for correcting camera frame C orientation
        T_CC = np.array([[ 1.000,  0.000,  0.000,  0.000],
                         [ 0.000, -1.000,  0.000,  0.000],
                         [ 0.000,  0.000, -1.000,  0.000],
                         [ 0.000,  0.000,  0.000,  1.000]])

        # Homogeneous transformation matrix from camera frame C to robot base frame R
        self.T_RC = T_RB @ T_BC @ T_CC

        # Timer for publishing repulsive force every 0.02 seconds
        self.timer = self.create_timer(0.02, self.publish_repulsive_force)

    def robot_state_callback(self, msg: FrankaRobotState):
        # Get end effector position
        x = msg.o_t_ee.pose.position.x
        y = msg.o_t_ee.pose.position.y
        z = msg.o_t_ee.pose.position.z
        self.end_effector_position = np.array([x, y, z])

        self.get_logger().info('End effector position: %s' % self.end_effector_position)

    def extract_point_cloud(self, point_cloud_camera: sl.Mat):
        # Extract point cloud data from ZED camera as X, Y, Z, RGBA values
        point_cloud_data = point_cloud_camera.get_data().reshape(-1, 4)

        # Randomly sample point cloud data by 2.0%
        valid_indices = self.indices[self.indices < len(point_cloud_data)]
        point_cloud_data = point_cloud_data[valid_indices]

        # Remove invalid data values (NaN values) from point cloud data
        mask = ~np.isnan(point_cloud_data).any(axis=1)
        point_cloud_data = point_cloud_data[mask]

        # Extract XYZ and RGBA values from point cloud data
        points_data = point_cloud_data[:, :3]
        rgba_data = point_cloud_data[:, 3:]

        # Extract RGB values from RGBA format
        rgba_data = np.frombuffer(rgba_data.tobytes(), dtype=np.uint32)
        r_data = (rgba_data >> 0) & 0xFF
        g_data = (rgba_data >> 8) & 0xFF
        b_data = (rgba_data >> 16) & 0xFF
        colors_data = np.column_stack((r_data, g_data, b_data)) / 255.0

        # Save point cloud data as Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points_data)
        point_cloud.colors = o3d.utility.Vector3dVector(colors_data)

        return point_cloud

    def remove_background(self, point_cloud: o3d.geometry.PointCloud):
        # Crop point cloud to workspace
        min_bound = np.array([-0.4, -0.9, -0.03])
        max_bound = np.array([1.2, 0.9, 1.4])
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        point_cloud = point_cloud.crop(bbox)

        return point_cloud

    def remove_robot_arm(self, point_cloud: o3d.geometry.PointCloud):
        # Split point cloud into smaller workspace and outside workspace
        min_bound = np.array([-0.4, -0.9, 0.03])
        max_bound = np.array([1.2, 0.9, 1.4])
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        indices_bbox = bbox.get_point_indices_within_bounding_box(point_cloud.points)
        point_cloud_workspace = point_cloud.select_by_index(indices_bbox, invert=False)
        point_cloud_outside = point_cloud.select_by_index(indices_bbox, invert=True)

        # Apply MinMaxScaler to normalize point cloud data
        data = np.asarray(point_cloud_workspace.points)
        scaler = MinMaxScaler(feature_range=(0, 1))
        scaler.fit([min_bound, max_bound])
        data = scaler.transform(data)

        # Apply DBSCAN clustering
        dbscan = DBSCAN(eps=0.034, min_samples=4)
        labels = dbscan.fit_predict(data)

        # Find robot arm label by searching for cluster containing end effector position
        point_cloud_tree = o3d.geometry.KDTreeFlann(point_cloud_workspace)
        robot_arm_point = self.end_effector_position + np.array([0.0, 0.0, 0.107])
        [_, idx, _] = point_cloud_tree.search_knn_vector_3d(robot_arm_point, 1)
        robot_arm_label = labels[idx[0]]

        # Remove robot arm from workspace
        mask = (labels != robot_arm_label)
        points_data = (np.asarray(point_cloud_workspace.points))[mask]
        colors_data = (np.asarray(point_cloud_workspace.colors))[mask]
        point_cloud_workspace.points = o3d.utility.Vector3dVector(points_data)
        point_cloud_workspace.colors = o3d.utility.Vector3dVector(colors_data)

        point_cloud_workspace, _ = point_cloud_workspace.remove_radius_outlier(nb_points=5, radius=0.05)

        return point_cloud_outside + point_cloud_workspace

    def add_obstacle(self, point_cloud: o3d.geometry.PointCloud):
        # Define dimensions of obstacle
        min_bound = np.array([0.4, 0.0, 0.0])
        max_bound = np.array([0.45, 0.05, 0.4])

        # Generate random points within obstacle dimensions and color them red
        volume = np.prod(max_bound - min_bound)
        num_points = int(volume * 1e5)
        points = np.random.uniform(min_bound, max_bound, size=(num_points, 3))
        colors = np.zeros((len(points), 3))
        colors[:, 0] = 1.0

        # Add red obstacle to point cloud
        obstacle = o3d.geometry.PointCloud()
        obstacle.points = o3d.utility.Vector3dVector(points)
        obstacle.colors = o3d.utility.Vector3dVector(colors)

        return point_cloud + obstacle

    def get_point_cloud(self):
        # Get point cloud data from ZED camera
        runtime_parameters = sl.RuntimeParameters()
        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            point_cloud_camera = sl.Mat()
            self.zed.retrieve_measure(point_cloud_camera, sl.MEASURE.XYZRGBA)
            point_cloud = self.extract_point_cloud(point_cloud_camera)
        else:
            self.get_logger().info('Error retrieving point cloud data, exit program')
            self.zed.close()
            exit(1)

        # Point cloud preprocessing
        point_cloud = point_cloud.transform(self.T_RC)
        point_cloud = self.remove_background(point_cloud)
        #point_cloud = self.add_obstacle(point_cloud)
        point_cloud = self.remove_robot_arm(point_cloud)

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

    def get_nearest_points(self, point_cloud: o3d.geometry.PointCloud):
        # Search nearest point in point cloud relative to end effector position with KDTree
        point_cloud_tree = o3d.geometry.KDTreeFlann(point_cloud)        
        [_, idx, dist] = point_cloud_tree.search_knn_vector_3d(self.end_effector_position, 1)
        obstacle = (np.asarray(point_cloud.points))[idx[0]]
        distance = np.sqrt(dist[0])

        # Search 20 nearest points in point cloud relative to nearest point with KDTree
        [k, idx, _] = point_cloud_tree.search_knn_vector_3d(obstacle, 20)
        if k != 0:
            k_obstacles = (np.asarray(point_cloud.points))[idx]
            k_distances = np.linalg.norm(self.end_effector_position - k_obstacles, axis=1)
        else:
            k_obstacles = obstacle
            k_distances = distance

        # Concatenate nearest point and k nearest points and distances
        obstacles = np.row_stack((obstacle, k_obstacles))
        distances = np.hstack((distance, k_distances))

        return obstacles, distances

    def compute_repulsive_force(self, obstacles, distances):
        # Define parameters for repulsive force calculation
        safe_distance = 0.3
        danger_distance = 0.15
        magnitude_limit = 50.0
        gain_repulsion = 20.0
        gain_tangential = 0.1

        # Compute repulsive force for k nearest points
        k = len(obstacles)
        F_repulsion = np.zeros((k, 3))
        mask = np.ones(k, dtype=bool)
        for i in range(k):
            if distances[i] < safe_distance:
                if distances[i] <= danger_distance:
                    distances[i] = danger_distance + 0.001
                    if np.array_equal(self.end_effector_position, obstacles[i]):
                        obstacles[i][2] = -0.001
                magnitude = gain_repulsion * np.log((safe_distance - danger_distance) / (distances[i] - danger_distance))
                unit_vector = (self.end_effector_position - obstacles[i]) / distances[i]
                F_repulsion[i] = magnitude * unit_vector
            else:
                F_repulsion[i] = np.zeros(3)
                mask[i] = False

        # Remove repulsive forces with zero magnitude
        F_repulsion = F_repulsion[mask]
        distances = distances[mask]

        # Compute weighted average of repulsive forces
        weights = distances / np.sum(distances)
        F_repulsion = np.dot(weights, F_repulsion)

        # Compute eigenvalues and eigenvectors of covariance matrix of obstacles
        mean = np.mean(obstacles, axis=0)
        data = obstacles - mean
        cov_matrix = np.cov(data, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # Sort eigenvalues and eigenvectors in ascending order
        sorted_indices = np.argsort(eigenvalues)
        eigenvalues = eigenvalues[sorted_indices]
        eigenvectors = eigenvectors[:, sorted_indices]

        # Find direction of minimum variance, which is not aligned with maximum direction of repulsive force
        direction_repulsion_index = np.argmax(F_repulsion)
        min_eigenvalue_index = np.argmin(eigenvalues)
        if min_eigenvalue_index == direction_repulsion_index:
            min_eigenvalue_index = (min_eigenvalue_index + 1) % 3
        min_eigenvector = eigenvectors[:, min_eigenvalue_index]

        # Compute unit vector as tangent to direction of minimum variance and aligned with repulsive force
        unit_vector = min_eigenvector
        unit_vector_inv = -1 * unit_vector
        proj_unit_vector = np.dot(F_repulsion, unit_vector)
        proj_unit_vector_inv = np.dot(F_repulsion, unit_vector_inv)
        if abs(proj_unit_vector_inv) > abs(proj_unit_vector):
            unit_vector = unit_vector_inv

        # Compute repulsive force with tangential component
        F_tangential = np.linalg.norm(F_repulsion) * unit_vector
        F_repulsion = (1 - gain_tangential) * F_repulsion + gain_tangential * F_tangential

        # Limit magnitude of repulsive force
        magnitude = np.linalg.norm(F_repulsion)
        if magnitude > magnitude_limit:
            F_repulsion = (magnitude_limit / magnitude) * F_repulsion

        return F_repulsion

    def publish_repulsive_force(self):
        # Get point cloud data
        point_cloud = self.get_point_cloud()

        # Publish point cloud data
        #self.publish_point_cloud(point_cloud)

        # Get nearest points and distances in point cloud relative to end effector position
        obstacles, distances = self.get_nearest_points(point_cloud)

        # Compute repulsive force
        F_repulsion = self.compute_repulsive_force(obstacles, distances)

        # Publish F_repulsion as WrenchStamped message
        msg = WrenchStamped()
        msg.wrench.force.x = F_repulsion[0]
        msg.wrench.force.y = F_repulsion[1]
        msg.wrench.force.z = F_repulsion[2]
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.publisher_repulsive_force.publish(msg)

        self.get_logger().info('F_repulsion: %s' % F_repulsion)

def main(args=None):
    rclpy.init(args=args)
    repulsive_force_publisher = RepulsiveForcePublisher()
    rclpy.spin(repulsive_force_publisher)
    repulsive_force_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

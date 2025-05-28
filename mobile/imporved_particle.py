import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
import yaml
import cv2
import os
from tf2_ros import TransformBroadcaster
from scipy.stats import norm
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter')
        self.best_pose_stamped = None

        self.declare_parameter('map_path', os.path.expanduser('~/map_217.yaml'))
        map_path = self.get_parameter('map_path').get_parameter_value().string_value

        self.map_data, self.resolution, self.origin = self.load_map(map_path)
        self.get_logger().info('Map loaded.')

        self.num_particles = 200
        self.min_particles = 20
        self.max_particles = 20
        self.ess_threshold = 0.5  # Effective Sample Size ratio threshold

        self.particles = self.generate_particles()
        self.prev_odom = None
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )

        self.particle_pub = self.create_publisher(PoseArray, '/particles', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.best_particle_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)

    def load_map(self, map_yaml_path):
        with open(map_yaml_path, 'r') as f:
            map_metadata = yaml.safe_load(f)

        image_path = os.path.join(os.path.dirname(map_yaml_path), map_metadata['image'])
        resolution = map_metadata['resolution']
        origin = map_metadata['origin']

        map_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        map_data = np.where(map_img < 127, 1, 0)
        return map_data, resolution, origin

    # def generate_particles(self):
    #     h, w = self.map_data.shape
    #     particles = []
    #     while len(particles) < self.num_particles:
    #         x = np.random.uniform(0, w * self.resolution)
    #         y = np.random.uniform(0, h * self.resolution)
    #         map_x = int((x - self.origin[0]) / self.resolution)
    #         map_y = int((y - self.origin[1]) / self.resolution)
    #         if 0 <= map_x < w and 0 <= map_y < h and self.map_data[map_y][map_x] == 0:
    #             theta = np.random.uniform(-np.pi, np.pi)
    #             particles.append((x, y, theta))
    #     return particles

    def generate_particles(self):
        particles = []
        center_x = 0.0  # 초기 위치 x
        center_y = 0.0  # 초기 위치 y
        h, w = self.map_data.shape

        while len(particles) < self.num_particles:
            x = np.random.normal(center_x, 0.1)  # 중심에서 약간의 노이즈
            y = np.random.normal(center_y, 0.1)
            map_x = int((x - self.origin[0]) / self.resolution)
            map_y = int((y - self.origin[1]) / self.resolution)

            if 0 <= map_x < w and 0 <= map_y < h and self.map_data[map_y][map_x] == 0:
                theta = 0.0  # 시작 방향을 0으로 고정
                particles.append((x, y, theta))

        return particles



    def odom_callback(self, msg):
        if self.prev_odom is None:
            self.prev_odom = msg
            return

        dx = msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        dtheta = self.get_yaw(msg.pose.pose.orientation) - self.get_yaw(self.prev_odom.pose.pose.orientation)
        self.motion_update(dx, dy, dtheta)
        self.prev_odom = msg

    def motion_update(self, delta_x, delta_y, delta_theta):
        new_particles = []
        for x, y, theta in self.particles:
            new_x = x + delta_x + np.random.normal(0, 0.02)
            new_y = y + delta_y + np.random.normal(0, 0.02)
            new_theta = theta + delta_theta + np.random.normal(0, 0.05)
            new_particles.append((new_x, new_y, new_theta))
        self.particles = new_particles

    def ray_casting(self, x, y, theta, angle, max_range=3.5):
        h, w = self.map_data.shape
        step_size = self.resolution / 2.0
        distance = 0.0
        ray_theta = theta + angle

        while distance < max_range:
            distance += step_size
            world_x = x + distance * math.cos(ray_theta)
            world_y = y + distance * math.sin(ray_theta)
            map_x = int((world_x - self.origin[0]) / self.resolution)
            map_y = int((world_y - self.origin[1]) / self.resolution)
            if 0 <= map_x < w and 0 <= map_y < h:
                if self.map_data[map_y][map_x] == 1:
                    break
            else:
                break
        return min(distance, max_range)

    def compute_particle_weight(self, particle, laser_msg, step=10, sigma=0.2):
        x, y, theta = particle
        ranges = np.array(laser_msg.ranges)
        angle_min = laser_msg.angle_min
        angle_increment = laser_msg.angle_increment
        weight = 1.0

        for i in range(0, len(ranges), step):
            real_range = ranges[i]
            if not (0.1 < real_range < laser_msg.range_max):
                continue
            beam_angle = angle_min + i * angle_increment
            expected_range = self.ray_casting(x, y, theta, beam_angle)
            error = real_range - expected_range
            weight *= norm.pdf(error, 0, sigma) + 1e-9
        return weight

    def laser_callback(self, msg):
        weights = np.array([self.compute_particle_weight(p, msg) for p in self.particles])
        weights += 1e-9
        weights /= np.sum(weights)

        ess = 1.0 / np.sum(weights ** 2)
        if ess / self.num_particles < self.ess_threshold:
            self.resample_particles(weights)

        self.publish_particles(self.particles, weights)
        best_particle = self.particles[np.argmax(weights)]
        self.publish_tf(best_particle)

    def resample_particles(self, weights):
        new_particles = []
        index = int(np.random.uniform(0, self.num_particles))
        beta = 0.0
        mw = np.max(weights)
        for _ in range(self.num_particles):
            beta += np.random.uniform(0, 2.0 * mw)
            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % self.num_particles
            new_particles.append(self.particles[index])
        self.particles = new_particles

    def publish_particles(self, particles, weights):
        msg = PoseArray()
        msg.header.frame_id = 'base_scan'
        msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y, theta) in particles:
            pose = Pose()
            pose.position.x = y
            pose.position.y = x
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            pose.orientation.z = qz
            pose.orientation.w = qw
            msg.poses.append(pose)

        self.particle_pub.publish(msg)

        # 🚀 Best Particle 선택 및 변환
        best_index = np.argmax(weights)
        best_particle = particles[best_index]
        best_pose_stamped = PoseStamped()
        best_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        best_pose_stamped.header.frame_id = "base_scan"
        best_pose_stamped.pose.position.x = best_particle[1]
        best_pose_stamped.pose.position.y = best_particle[0]
        print(best_particle[0])
        print(best_particle[1])
        best_pose_stamped.pose.position.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_yaw(best_particle[2])
        best_pose_stamped.pose.orientation.x = qx
        best_pose_stamped.pose.orientation.y = qy
        best_pose_stamped.pose.orientation.z = qz
        best_pose_stamped.pose.orientation.w = qw

        # ✅ 속성을 올바르게 설정
        self.best_pose_stamped = best_pose_stamped

        # ✅ PoseStamped → PoseWithCovarianceStamped 변환 및 퍼블리시
        best_pose_covariance = self.convert_pose_stamped_to_covariance(self.best_pose_stamped)
        self.best_particle_pub.publish(best_pose_covariance)



    def publish_tf(self, best_particle):
        x, y, theta = best_particle
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_scan"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def get_yaw(self, orientation):
        qz = orientation.z
        qw = orientation.w
        return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

    def quaternion_from_yaw(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw
    def convert_pose_stamped_to_covariance(self,pose_stamped):
        pose_covariance = PoseWithCovarianceStamped()
        pose_covariance.header = pose_stamped.header
        pose_covariance.pose.pose = pose_stamped.pose
        pose_covariance.pose.covariance = [0.0] * 36  # 기본값 설정 (필요하면 수정 가능)
        return pose_covariance

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

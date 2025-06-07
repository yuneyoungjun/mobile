import rclpy
from rclpy.node import Node
import numpy as np
import onnxruntime as ort
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

class TurtleBotController(Node):
    def __init__(self, onnx_model_path):
        super().__init__('turtlebot_controller')

        # ✅ ONNX 모델 로드
        self.session = ort.InferenceSession(onnx_model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.get_logger().info(f"ONNX Model Loaded: {onnx_model_path}")

        # ✅ ROS 2 토픽 설정
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(MarkerArray, '/waypoints', self.control_turtlebot, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ✅ 변수 초기화
        self.lidar_data = np.zeros(121, dtype=np.float32)
        self.robot_position = np.zeros(3, dtype=np.float32)
        self.robot_forward = np.zeros(3, dtype=np.float32)
        self.target_position = np.zeros(3, dtype=np.float32)
        self.input_vector = np.zeros(130, dtype=np.float32)
        self.history = []
        # self.v_sale = 0.09
        self.v_sale = 0.2
        # self.w_sale = 0.3
        self.w_sale =0.2

        # ✅ 최근 속도 저장용 변수
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # ✅ 20Hz 주기로 cmd_vel 전송ddddd
        self.timer = self.create_timer(0.05, self.timer_callback)

    def pose_callback(self, msg):
        """ ✅ 로봇 위치 및 방향 업데이트 """
        self.robot_position = np.array([
            -msg.pose.pose.position.y,
            msg.pose.pose.position.x,
            0
        ], dtype=np.float32)

        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        # yaw=(yaw+np.pi)%np.pi-np.pi
        yaw=-yaw
        self.robot_forward = np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=np.float32)

    def lidar_callback(self, msg):
        """ ✅ -60도에서 60도 사이 121개 샘플을 저장 """
        ranges = np.array(msg.ranges, dtype=np.float32)

        part1 = ranges[300:359]     # 0 ~ 60 (61개)
        part2 = ranges[0:60]  # 300 ~ 359 (60개)

        filtered_ranges = np.concatenate((part1, part2))
        self.lidar_data = np.nan_to_num(filtered_ranges, nan=10.0, posinf=10.0, neginf=0.0)

    def control_turtlebot(self, msg: MarkerArray):
        """ ✅ MarkerArray 기반 제어 및 ONNX 모델 실행 """
        if msg.markers:
            target_marker = msg.markers[-1]
            self.target_position = np.array([
                target_marker.pose.position.x,
                target_marker.pose.position.y,
                0
            ], dtype=np.float32)

                # ✅ 입력 벡터 생성
        self.input_vector[:121] = self.lidar_data * 15
        self.input_vector[121:124] = self.robot_forward
        self.input_vector[124:127] = self.robot_position
        self.input_vector[127:130] = self.target_position

        self.history.append(self.input_vector.copy())
        if len(self.history) > 10:
            self.history.pop(0)

        if len(self.history) < 10:
            return

        model_input = np.concatenate(self.history, axis=None).reshape(1, 1300).astype(np.float32)

        outputs = self.session.run(None, {self.input_name: model_input})

        if len(msg.markers) != 0:
            self.linear_velocity = max(min(outputs[2][0][1] * self.v_sale, 0.1), -0.1)+0.05
            self.angular_velocity = max(min(outputs[2][0][0] * self.w_sale, 0.2), -0.2)
            # if self.linear_velocity <=0:
            #     self.angular_velocity=-self.angular_velocity

        else:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        self.get_logger().info(
            f"Updated velocity: v={self.linear_velocity:.3f}, w={self.angular_velocity:.3f}"
        )

    def timer_callback(self):
        """ ✅ 주기적으로 cmd_vel publish하여 부드러운 이동 유도 """
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear_velocity)
        twist_msg.angular.z = float(self.angular_velocity)
        self.vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    onnx_model_path = "/home/yuneyoungjun/onnx_train/Ray.onnx"
    node = TurtleBotController(onnx_model_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()








































# import rclpy
# from rclpy.node import Node
# import numpy as np
# import onnxruntime as ort
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray,PointStamped
# from tf_transformations import euler_from_quaternion
# from rclpy.qos import qos_profile_sensor_data

# class TurtleBotController(Node):
#     def __init__(self, onnx_model_path):
#         super().__init__('turtlebot_controller')

#         # ✅ ONNX 모델 로드
#         self.session = ort.InferenceSession(onnx_model_path)
#         self.input_name = self.session.get_inputs()[0].name
#         self.input_shape = self.session.get_inputs()[0].shape
#         self.get_logger().info(f"ONNX Model Loaded: {onnx_model_path}")

#         # ✅ ROS 2 토픽 설정
#         self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
#         self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
#         # self.create_subscription(PoseArray, '/waypoints', self.control_turtlebot, 10)
#         self.create_subscription(PointStamped, '/clicked_point', self.control_turtlebot, 10)
#         self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # ✅ 변수 초기화
#         self.lidar_data = np.zeros(121, dtype=np.float32)
#         self.robot_position = np.zeros(3, dtype=np.float32)
#         self.robot_forward = np.zeros(3, dtype=np.float32)
#         self.target_position = np.zeros(3, dtype=np.float32)
#         self.input_vector = np.zeros(130, dtype=np.float32)
#         self.history = []
#         self.v_sale = 0.2
#         self.w_sale = 0.5

#         # ✅ 최근 속도 저장용 변수
#         self.linear_velocity = 0.0
#         self.angular_velocity = 0.0

#         # ✅ 20Hz 주기로 cmd_vel 전송
#         self.timer = self.create_timer(0.05, self.timer_callback)

#     def pose_callback(self, msg):
#         """ ✅ 로봇 위치 및 방향 업데이트 """
#         self.robot_position = np.array([
#             msg.pose.pose.position.x,
#             msg.pose.pose.position.y,
#             0
#         ], dtype=np.float32)

#         orientation_q = msg.pose.pose.orientation
#         (_, _, yaw) = euler_from_quaternion([
#             orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
#         ])

#         self.robot_forward = np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=np.float32)

#     def lidar_callback(self, msg):
#         """ ✅ -60도에서 60도 사이 121개 샘플을 저장 """
#         ranges = np.array(msg.ranges, dtype=np.float32)

#         part1 = ranges[300:360]  # 300 ~ 359 (60개)
#         part2 = ranges[0:61]     # 0 ~ 60 (61개)

#         filtered_ranges = np.concatenate((part1, part2))
#         self.lidar_data = np.nan_to_num(filtered_ranges, nan=10.0, posinf=10.0, neginf=0.0)

#     def control_turtlebot(self, msg):
#         """ ✅ 웨이포인트 기반 제어 및 ONNX 모델 실행 """
#         # if msg.poses:
#         #     target_pose = msg.poses[-1]
#         #     self.target_position = np.array([
#         #         target_pose.position.x,
#         #         target_pose.position.y,
#         #         0
#         #     ], dtype=np.float32)

#         # ✅ 입력 벡터 생성
#         """ ✅ 웨이포인트 기반 제어 및 ONNX 모델 실행 """
#         if msg.point:
#             target_pose = msg.point
#             self.target_position = np.array([
#                 target_pose.x,
#                 target_pose.y,
#                 0
#             ], dtype=np.float32)
#         print(target_pose)
#         self.input_vector[:121] = self.lidar_data * 30
#         self.input_vector[121:124] = self.robot_forward
#         self.input_vector[124:127] = self.robot_position
#         self.input_vector[127:130] = self.target_position

#         self.history.append(self.input_vector.copy())
#         if len(self.history) > 10:
#             self.history.pop(0)

#         if len(self.history) < 10:
#             return

#         model_input = np.concatenate(self.history, axis=None).reshape(1, 1300).astype(np.float32)

#         outputs = self.session.run(None, {self.input_name: model_input})
#         if len(msg.poses) != 0:
#             self.linear_velocity = max(min(outputs[2][0][1] * self.v_sale, 0.5), -0.5)
#             self.angular_velocity = max(min(outputs[2][0][0] * self.w_sale, 0.5), -0.5)
#         else:
#             self.linear_velocity = 0.0
#             self.angular_velocity = 0.0

#         self.get_logger().info(
#             f"Updated velocity: v={self.linear_velocity:.3f}, w={self.angular_velocity:.3f}"
#         )

#     def timer_callback(self):
    
#         """ ✅ 주기적으로 cmd_vel publish하여 부드러운 이동 유도 """
#         twist_msg = Twist()
#         twist_msg.linear.x = float(self.linear_velocity)
#         twist_msg.angular.z = float(self.angular_velocity)
#         self.vel_pub.publish(twist_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     onnx_model_path = "/home/yuneyoungjun/onnx_train/Ray.onnx"
#     node = TurtleBotController(onnx_model_path)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




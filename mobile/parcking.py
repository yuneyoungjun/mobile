import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion

# class DifferentialDriveRobot(Node):
#     def __init__(self, x0, y0, theta0, goal_pos, goal_theta=0.0,
#                  k_rho=3.0, k_alpha=10.0, k_beta=-1.0, dt=0.01):
#         super().__init__('parking_node')
#         self.x = x0
#         self.y = y0
#         self.theta = theta0
#         self.goal_pos = goal_pos
#         self.goal_theta = goal_theta
#         self.k_rho = k_rho
#         self.k_alpha = k_alpha
#         self.k_beta = k_beta
#         self.dt = dt
#         self.trajectory = [[self.x, self.y]]
#         self.orientations = [self.theta]

#         self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.step, 10)
#         self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)


#     def step(self,msg):
#         def pp(x):
#             return (x+np.pi)%2*np.pi-np.pi
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         orientation_q = msg.pose.pose.orientation
#         (_, _, psi) = euler_from_quaternion([
#             orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
#         ])
        
#         self.get_logger().info(f"x: {x}, y: {y}, psi: {psi}")
#         dx = self.goal_pos[0] - x
#         dy = self.goal_pos[1] - y
#         self.theta=psi*180/np.pi
#         rho = np.hypot(dx, dy)
#         angle_to_goal = np.arctan2(dy, dx)
#         alpha = angle_to_goal - self.theta
#         beta = self.goal_theta - angle_to_goal

#         # alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
#         # beta = np.arctan2(np.sin(beta), np.cos(beta))

#         forward = True
#         first=0
#         first+=1
#         if abs(alpha) > np.pi / 2 :
#             alpha = np.arctan2(np.sin(alpha + np.pi), np.cos(alpha + np.pi))
#             beta = np.arctan2(np.sin(beta + np.pi), np.cos(beta + np.pi))
#             forward = False
        

#         v = 1-1/(self.k_rho * rho+1)
#         if not forward:
#             v *= -1
#         w = 1-1/(self.k_alpha * alpha + self.k_beta * beta+1)
#         scale_v=2
#         scale_w=1
#         v = self.k_rho * rho
#         w = self.k_alpha * alpha + self.k_beta * beta
#         w*=scale_w
#         v*=scale_v
#         self.get_logger().info(f"v: {v}, w: {w}")
#         twist_msg = Twist()
#         twist_msg.linear.x = v  # x 방향으로 전진 속도
#         twist_msg.angular.z = w  # z 방향으로 회전 속도
#         self.vel_pub.publish(twist_msg)

#         return


# def main(args=None):
#     rclpy.init(args=args)
#     node = DifferentialDriveRobot(0, 0, 0, [2, 2], 0.0)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




class DifferentialDriveRobot(Node):
    def __init__(self, x0, y0, theta0, goal_pos, goal_theta=0.0,
                 k_rho=3.0, k_alpha=8.0, k_beta=-1.5, dt=0.01):
        super().__init__('parking_node')
        self.x = x0
        self.y = y0
        self.theta = theta0
        self.goal_pos = goal_pos
        self.goal_theta = goal_theta
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta
        self.dt = dt
        self.trajectory = [[self.x, self.y]]
        self.orientations = [self.theta]
        self.mission=0

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.step, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ✅ 이전 속도를 저장할 변수 추가 (초기값은 정지)
        self.prev_twist = Twist()
        self.prev_twist.linear.x = 0.0
        self.prev_twist.angular.z = 0.0

        # ✅ 주기적으로 퍼블리시할 타이머 추가 (50ms마다 실행)
        self.timer = self.create_timer(0.05, self.publish_twist)

    # def step(self, msg):
    #     def pp(x):
    #         return (x + np.pi) % (2 * np.pi) - np.pi

    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     orientation_q = msg.pose.pose.orientation
    #     (_, _, psi) = euler_from_quaternion([
    #         orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    #     ])
        
    #     # ✅ 로봇 방향 업데이트 (라디안 유지)
    #     self.theta = psi

    #     dx = self.goal_pos[0] - x
    #     dy = self.goal_pos[1] - y
    #     rho = np.hypot(dx, dy)
    #     angle_to_goal = np.arctan2(dy, dx)

    #     alpha = angle_to_goal - psi
    #     beta = pp(self.goal_theta - angle_to_goal)

    #     forward = True
    #     if abs(alpha) > np.pi / 2:
    #         alpha = pp(alpha + np.pi)
    #         beta = pp(beta + np.pi)
    #         forward = not forward  # 방향 반전
        
    #     v = self.k_rho * rho
    #     if not forward:
    #         v *= -1
    #     w = self.k_alpha * alpha + self.k_beta * beta

    #     scale_v = 0.05
    #     scale_w = 0.1
    #     w *= scale_w
    #     v *= scale_v
    #     self.get_logger().info(f"x: {x}, y: {y}, psi: {psi}, alpha: {angle_to_goal}")
    #     self.get_logger().info(f"v: {v}, w: {w}")

    #     # ✅ 새로운 속도로 업데이트
    #     self.prev_twist.linear.x = v
    #     self.prev_twist.angular.z = w
    




    def step(self, msg):
        def pp(x):
            return (x + np.pi) % (2 * np.pi) - np.pi
        def wrap_to_2pi(angle):
            return (angle + 2 * np.pi) % (2 * np.pi)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, psi) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        print(psi)
        
        # ✅ 로봇 방향 업데이트
        self.theta = psi

        dx = self.goal_pos[0] - x
        dy = self.goal_pos[1] - y
        rho = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)

        alpha = angle_to_goal - psi
        beta = pp(self.goal_theta - psi)  # ✅ 목표 방향 정렬을 정확하게 수정!

        forward = True
        if abs(alpha) > np.pi / 2:
            alpha = alpha + np.pi
            beta = beta + np.pi
            # forward = not forward  # ✅ 방향 반전
        
        v = self.k_rho * rho * np.exp(-rho)  # ✅ 목표 위치 근처에서 속도 줄이기
        if not forward:
            v *= -1

        # alpha *= np.exp(-rho)  # ✅ 목표 위치 근처에서 alpha 영향 줄이기
        # beta *= 1.5 - np.exp(-rho)  # ✅ 목표 위치 근처에서 beta 영향 증가

        w = self.k_alpha * alpha + self.k_beta * beta

        scale_v =1
        scale_w = 1
        # w *= scale_w* (1-np.exp(-rho))
        # v *= scale_v* (1-np.exp(-rho))
        w *= scale_w* (1-np.exp(-rho))
        v *= scale_v* (1-np.exp(-rho))
        w=min(3,w)
        v=min(3,v)

        self.get_logger().info(f"x: {x}, y: {y}, psi: {psi}, alpha: {alpha}, beta: {beta}")
        self.get_logger().info(f"v: {v}, w: {w}")

        # ✅ 새로운 속도로 업데이트
        self.prev_twist.linear.x = v
        self.prev_twist.angular.z = w
        if( rho<1 and self.mission==0):
            self.mission=1
            # self.prev_twist.linear.x = 0.0
            # w = -self.k_beta * beta*scale_w
            # self.prev_twist.angular.z = 0.0

        # else:
        #     self.mission=0

        if( self.mission==1):
            self.prev_twist.linear.x = 0.0
            w = -self.k_beta * (self.goal_theta - psi)
            w *= scale_w*2/3
            self.prev_twist.angular.z=w
            if( abs(self.goal_theta - psi) < 0.1):
                self.mission=2
                print(w)
        if( self.mission==2):
            self.prev_twist.linear.x = 0.0
            self.prev_twist.angular.z = 0.0
            self.get_logger().info("finish")


    def publish_twist(self):
        # ✅ 항상 마지막 속도를 퍼블리시
        self.vel_pub.publish(self.prev_twist)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveRobot(0, 0, 0, [3, 0], 179*np.pi/180)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

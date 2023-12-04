import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

class KalmanFilter:

    def __init__(self, dt, A, C, Q, R, P):
        self.A = A
        self.C = C
        self.Q = Q
        self.R = R
        self.P = P
        self.K = np.zeros_like(P)
        self.P0 = P
        self.m, self.n = C.shape
        self.t0 = 0
        self.t = 0
        self.dt = dt
        self.initialized = False
        self.I = np.eye(self.n)
        self.x_hat = np.zeros((self.n, 1))
        self.x_hat_new = np.zeros((self.n, 1))

    def init(self, t0=0, x0=None):
        self.t0 = t0
        self.t = t0
        self.x_hat = x0 if x0 is not None else np.zeros((self.n, 1))
        self.P = self.P0
        self.initialized = True

    def update(self, y, dt=None, A=None):
        if not self.initialized:
            raise Exception("Filter not initialized!")

        if dt is None:
            dt = self.dt
        if A is None:
            A = self.A

        # Prediction
        x_hat_minus = A @ self.x_hat
        P_minus = A @ self.P @ A.T + self.Q

        # Update
        S = self.C @ P_minus @ self.C.T + self.R
        self.K = P_minus @ self.C.T @ np.linalg.inv(S)
        self.x_hat_new = x_hat_minus + self.K @ (y - self.C @ x_hat_minus)
        self.P = (self.I - self.K @ self.C) @ P_minus

        # Update time and state
        self.t += dt
        self.x_hat = self.x_hat_new

        return self.x_hat.flatten(), self.t

class DepthKalman(Node):
    lista =[]
    ian = 0
    
    def __init__(self):
        super().__init__('depth_kalman')
        # Create publisher
        self.motor_pid_pub_ = self.create_publisher(Float64, 'depth', 10)
        self.depth_pub_ = self.create_publisher(Float64, 'depth_raw', 10)
        # Create subscriber
        self.robot_mode_sub_ = self.create_subscription(Vector3, 'human_pose', self.depth_callback, 10)
        self.robot_mode_sub_  # prevent unused variable warning

        # Create KalmanFilter object
        dt = 0.03
        # Your matrices
        # 3*3
        A = np.array([[1, dt],
                      [1, 0]])
        # 1*3
        C = np.array([[1, 0]])
        # 3*3
        Q = np.diag([0.1, 0])
        # 1
        R = np.array([[0.15]])
        # 3*3
        P = np.array([[0.6, 0], 
                      [0, 0.4]])
        self.kf = KalmanFilter(dt, A, C, Q, R, P)

        x0 = np.array([0.8, 0])
        self.kf.init(x0=x0)

    def depth_callback(self, msg):
        depth = (msg.y)/1000.0
        # Publish Robot_state
        # self.lista.append(depth)
        # self.ian += 1
        # if self.ian==20:
        #     self.ian=0
        #     self.get_logger().info("Cov:%f"%np.cov(self.lista))
        #     self.lista= []
        pub_msg = Float64()
        pub__raw_msg = Float64()
        pub__raw_msg.data = float(depth)
        x_hat, t = self.kf.update(depth)
        s = round(float(x_hat[0]), 1)
        if s<=0:
            s=0.0
        pub_msg.data =s
        # s = depth/1000.0
        # publish data
        # self.get_logger().info("estimate: '%f'"%s)
        self.motor_pid_pub_.publish(pub_msg)
        self.depth_pub_.publish(pub__raw_msg)

def main(args=None):
    rclpy.init(args=args)
    depth_kalman = DepthKalman()
    rclpy.spin(depth_kalman)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_kalman.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
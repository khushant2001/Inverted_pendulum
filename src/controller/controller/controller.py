# Node that actuates the prismatic joint. 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import QoSProfile
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

class actuation(Node):
    def __init__(self):
        super().__init__("Controller")
        qos_profil = QoSProfile(depth=10)
        self.imu_data = self.create_subscription(Imu,'/imu',self.imu_callback,qos_profile=qos_profil)
        self.prismatic_state = self.create_subscription(JointState,'/prismatic_joint/state',self.prismatic_callback,qos_profile=qos_profil)
        self.actuation = self.create_publisher(Float64,'/pendulum_joint_command',qos_profile=qos_profil)
        self.timer_period = 0.002
        self.timer = self.create_timer(self.timer_period, self.control)

        # Defining variables to use later!
        self.roll_angle = 0
        self.roll_vel = 0
        self.prismatic_x = 0
        self.prismatic_x_vel = 0

        # PI Control control variables
        self.integral_sum = 0

    def imu_callback(self,msg:Imu):
        self.get_logger().info("Getting the pose of the pendulum!")
        
        # Extract quaternions from the message
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Convert from quaternion to euler angles
        roll_imu, pitch_imu, yaw_imu = self.quaternion_to_euler(w,x,y,z)

        # Convert from local to global frame. 
        # Pendulum's orientation (from the SDF pose)
        roll_pendulum = 0.1  # Replace with the actual roll from the SDF pose
        pitch_pendulum = 0.0  # Replace with the actual pitch from the SDF pose
        yaw_pendulum = 0.0  # Replace with the actual yaw from the SDF pose

        # Convert the pendulum's Euler angles to a rotation matrix
        R_pendulum = R.from_euler('xyz', [roll_pendulum, pitch_pendulum, yaw_pendulum]).as_matrix()

        # Convert IMU's Euler angles to a rotation matrix
        R_imu = R.from_euler('xyz', [roll_imu, pitch_imu, yaw_imu]).as_matrix()

        # Multiply the pendulum rotation matrix with the IMU rotation matrix to get the global rotation
        R_global = np.dot(R_pendulum, R_imu)

        # Convert the global rotation matrix back to Euler angles (roll, pitch, yaw)
        roll, _,_ = R.from_matrix(R_global).as_euler('xyz', degrees=True)
        
        # Angular velocity in the IMU (local) frame
        ang_vel_local = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Convert angular velocity to global/world frame
        ang_vel_global = R_imu @ ang_vel_local

        # Roll velocity in the global frame (x-axis)
        self.roll_vel = ang_vel_global[0]

        # Update the class variable
        self.roll_angle = roll
        self.get_logger().info(f'Roll angle = {roll}, roll velocity = {self.roll_vel}')

    def prismatic_callback(self, msg = JointState):

        # Update the class variables!
        self.prismatic_x = msg._position[0]
        self.prismatic_x_vel = msg.velocity[0]

        self.get_logger().info(f'Prismatic joint pos = {self.prismatic_x}, vel = {self.prismatic_x_vel}')
    
    # Get the acutation from the PI controller
    def control(self):
        msg = Float64()

        # Apply PI control
        #msg.data = self.pi(-self.roll_angle)

        # Apply LQR control!
        msg.data = self.lqr()

        # Publish and log the message!
        self.actuation.publish(msg)
        self.get_logger().info(f'Actuating = {msg.data}')

    # Designing the PI controller!
    def pi(self, error):
        kp = .27 # Proportional gain
        ki = .26 # Integral gain
        self.integral_sum = self.integral_sum + error*self.timer_period
        control = kp * error + ki * self.integral_sum
        return control
    
    def lqr(self):

        # Defining the lqr gains => K
        K = np.array([
            [0, 1.5, 0, 0.4]
        ])
        
        # Define the state
        state = np.array([
            [self.prismatic_x],[self.roll_angle],[self.prismatic_x_vel],[self.roll_vel]
        ])
        control = -K @ state
        return control.item()
    # Defining the translation from quaternion to euler!
    def quaternion_to_euler(self, w, x, y, z):
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args = None):
    rclpy.init(args = args)
    node = actuation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == 'main':
    main()
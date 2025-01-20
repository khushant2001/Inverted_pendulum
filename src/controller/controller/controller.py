# Node that actuates the prismatic joint. 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

class actuation(Node):
    def __init__(self):
        super().__init__("Controller")
        qos_profil = QoSProfile(depth=10)
        self.imu_data = self.create_subscription(Imu,'/imu',self.imu_callback,qos_profile=qos_profil)
        self.actuation = self.create_publisher(Float64,'/pendulum_joint_command',qos_profile=qos_profil)
        self.timer_period = 0.002
        self.timer = self.create_timer(self.timer_period, self.control)
        self.roll_angle = 0

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
        roll, pitch, yaw = R.from_matrix(R_global).as_euler('xyz', degrees=True)

        # Update the class variable
        self.roll_angle = roll
        self.get_logger().info(f'Global Angles = {roll}, {pitch}, {yaw}')

    # Get the acutation from the PI controller
    def control(self):
        msg = Float64()
        msg.data = self.pid(-self.roll_angle)
        self.actuation.publish(msg)
        self.get_logger().info(f'Actuating = {msg.data}')

    # Designing the PI controller!
    def pid(self, error):
        kp = .27 # Proportional gain
        ki = .26 # Integral gain
        self.integral_sum = self.integral_sum + error*self.timer_period
        control = kp * error + ki * self.integral_sum
        return control
    
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
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class actuation(Node):
    def __init__(self):
        super().__init__("Controller")
        qos_profil = QoSProfile(depth=10)
        self.imu_data = self.create_subscription(Imu,'/imu',self.imu_callback,qos_profile=qos_profil)
        self.actuation = self.create_publisher(Float64,'/pendulum_joint_command',qos_profile=qos_profil)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.control)
        self.roll_angle = 0

    def imu_callback(self,msg:Imu):
        self.get_logger().info("Getting the pose of the pendulum!")
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        roll, pitch, yaw = quaternion_to_euler(w,x,y,z)

        # Update the class variable!
        self.roll_angle = roll
        self.get_logger().info(f'Angles = {roll}, {pitch}, {yaw}')

    def control(self):
        msg = Float64()
        msg.data = pid(3.14 - self.roll_angle)
        self.actuation.publish(msg)
        self.get_logger().info("Actuating")

# Defining the translation from quaternion to euler!
def quaternion_to_euler(w,x, y, z):
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

# Defining arrays to store the integral and error terms for pid controller!
error_array = []
integral = [0]

# Designing the pid controller!
def pid(error):
    time_step = 0.02
    kp = 10 # Proportional gain
    ki = .2 # Integral gain
    kd = 0 # Derivative gain
    error_array.append(error)
    integral.append(error_array[-1]*time_step + integral[-1])
    derivative = (error_array[-1] - error_array[-2]) / time_step if len(error_array) > 1 else 0
    control = kp * error_array[-1] + ki * integral[-1] + kd * derivative
    return control

def main(args = None):
    rclpy.init(args = args)
    node = actuation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == 'main':
    main()

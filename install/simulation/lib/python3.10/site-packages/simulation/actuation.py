import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import math

class actuation(Node):
    def __init__(self):
        super().__init__("Actuation")
        self.tf = self.create_subscription(TFMessage,"/tf",self.pendulum_pose)
        self.actuation = self.create_publisher(JointState,"/joint_states",self.control)
        self.roll_angle = None

    def control(self):
        self.get_logger().info("Giving actuation to the pendulum!")
        msg = JointState()
        print(msg._velocity)
        #self.actuation.publish(msg)

    def pendulum_pose(self,msg:TFMessage):
        self.get_logger().info("Getting the pose of the pendulum!")
        print(msg._transforms)

# Defining the translation from quaternion to euler!

def quaternion_to_euler(x, y, z, w):
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
integral = []

# Designing the pid controller!
def pid(error):
    kp = 1 # Proportional gain
    ki = 1 # Integral gain
    kd = 1 # Derivative gain
    error_array.append(error)
    integral.append(error_array[-1]*time_step + integral[-1])
    derivative = (error_array[-1] - error_array[-2]) / time_step if len(error) > 1 else 0
    control = kp * error[-1] + ki * integral[-1] + kd * derivative
    return control

def main(args = None):
    rclpy.init(args = args)
    node = actuation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == 'main':
    main()
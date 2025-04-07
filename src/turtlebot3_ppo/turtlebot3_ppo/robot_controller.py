import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Subscription to the PoseStamped topic        
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST)
        self.qualisys_subscriber = self.create_subscription(PoseStamped, '/qualysis/tb3_3', self.qualisys_callback, qos)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID parameters for linear and angular velocities
        self.kp_linear = 1.0
        self.kp_angular = 6.0

        # Initialize PID variables
        self.threshold_distance = 0.05  # Threshold to stop

        # Target goal
        self.goal_x = 1.0
        self.goal_y = 1.0

    def angle_difference(self, desired_angle, current_angle):
        error = desired_angle - current_angle
        return (error + math.pi) % (2 * math.pi) - math.pi

    def navigate_to_goal(self, current_x, current_y, current_theta):
        # Compute distance error
        error_x = self.goal_x - current_x
        error_y = self.goal_y - current_y
        distance_error = math.hypot(error_x, error_y)
        target_theta = math.atan2(error_y, error_x)

        # Compute heading error
        heading_error = self.angle_difference(target_theta, current_theta)

        self.get_logger().info(f"Distance error [m]: {distance_error}")
        self.get_logger().info(f"Heading error [degrees]: {heading_error * 180 / math.pi}")

        if distance_error <= self.threshold_distance:
            return 0.0, 0.0  # Stop if goal is reached

        # Control commands
        control_linear = self.kp_linear * distance_error
        control_angular = self.kp_angular * heading_error

        return control_linear, control_angular

    def qualisys_callback(self, msg):
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_theta = msg.pose.orientation.z

        print(current_x, current_y)

        # Get control commands
        control_linear, control_angular = self.navigate_to_goal(current_x, current_y, current_theta)

        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = control_linear
        twist_msg.angular.z = control_angular

        # Limit velocities
        max_linear_speed = 0.21  # Max linear speed of TurtleBot3
        max_angular_speed = 0.85  # Max angular speed of TurtleBot3

        twist_msg.linear.x = max(min(twist_msg.linear.x, max_linear_speed), -max_linear_speed)
        twist_msg.angular.z = max(min(twist_msg.angular.z, max_angular_speed), -max_angular_speed)

        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import tf_transformations

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Assuming you are subscribing to the /odom topic for pose info
            self.pose_callback,
            10)

        self.current_pose = None
        self.target_pose = [2.0, 2.0]  # Example target in meters (x, y)
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.current_pose is None:
            return

        # Get current position
        x = self.current_pose.position.x
        y = self.current_pose.position.y

        # Extract yaw from quaternion (orientation)
        q = self.current_pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Calculate the distance and angle to the target
        dx = self.target_pose[0] - x
        dy = self.target_pose[1] - y

        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # Calculate yaw error (angle between current orientation and desired direction)
        yaw_error = angle_to_goal - yaw
        # Normalize yaw error to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        vel_msg = Twist()

        # If yaw error is significant, rotate the robot to face the goal
        if abs(yaw_error) > 0.1:
            vel_msg.angular.z = 0.5 * yaw_error
        # If not facing the goal, move forward
        elif distance > 0.05:
            vel_msg.linear.x = 0.2 * distance  # Adjust speed proportional to distance
        else:
            # If target is reached (distance is small enough), stop the robot
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        # Publish the velocity command
        self.publisher_.publish(vel_msg)

        # Print for debugging
        print(f"dx: {dx}, dy: {dy}, yaw_error: {yaw_error}, vel: {vel_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

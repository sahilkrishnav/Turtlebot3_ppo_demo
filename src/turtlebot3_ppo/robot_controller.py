import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.subscription = self.create_subscription(
            PoseStamped,
            'qualysis/tb3_3',  # or whatever your marker_deck_name is
            self.pose_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.current_pose = None
        self.target_pose = [1.0, 1.0]  # Example target in meters
        self.timer = self.create_timer(0.1, self.control_loop)  

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y

        yaw = self.current_pose.pose.orientation.z  # assuming yaw was stored in `.z`

        dx = self.target_pose[0] - x
        dy = self.target_pose[1] - y

        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        vel_msg = Twist()

        # Turn toward the goal
        yaw_error = angle_to_goal - yaw
        if abs(yaw_error) > 0.1:
            vel_msg.angular.z = 0.3 * yaw_error
        else:
            # Move forward
            vel_msg.linear.x = 0.5 * distance

        self.publisher_.publish(vel_msg)

def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

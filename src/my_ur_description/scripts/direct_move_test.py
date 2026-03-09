import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup

class UR12eDirectTest(Node):
    def __init__(self):
        super().__init__('ur12e_direct_test')
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Direct Move Test Initialized...")

    def move_to_pose(self, x, y, z, theta_deg):
        theta_rad = math.radians(theta_deg)
        self.get_logger().info(f"Targeting -> X:{x} Y:{y} Z:{z} Theta:{theta_deg}")

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Define the target pose
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        
        # Neutral "Pointing Down" orientation + Theta
        target.pose.orientation.x = 1.0 * math.sin(theta_rad/2)
        target.pose.orientation.y = 1.0 * math.cos(theta_rad/2)
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        # Simple Pose Goal (Less restrictive than PositionConstraints)
        from moveit_msgs.msg import Constraints, JointConstraint # Using standard pose goal
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        
        # Add the pose goal to the request
        from moveit_msgs.msg import Constraints
        goal_msg.request.goal_constraints.append(Constraints())
        # Note: In pure Action calls, we usually wrap the pose in a constraint 
        # but for this debug, let's use the simplest version
        
        self.get_logger().info("Waiting for MoveGroup server...")
        self.move_group_client.wait_for_server()
        
        self.get_logger().info("Sending Goal...")
        self._send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("GOAL REJECTED by MoveIt (Likely Collision or Unreachable)")
            return
        self.get_logger().info("Goal Accepted! Moving...")

def main():
    rclpy.init()
    node = UR12eDirectTest()
    # Test the specific position you requested
    node.move_to_pose(-0.051, 0.922, 0.502, 90.0)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
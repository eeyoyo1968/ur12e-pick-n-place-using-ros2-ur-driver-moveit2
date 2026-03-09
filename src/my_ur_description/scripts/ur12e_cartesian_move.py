import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
# We use the MoveGroup action to send Cartesian goals
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint

class UR12eCartesianMover(Node):
    def __init__(self):
        super().__init__('ur12e_cartesian_mover')
        # This client talks to MoveIt 2
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def move_to_xyz(self, x, y, z, theta_z):
        self._action_client.wait_for_server()
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_arm" # Check your MoveIt config for the group name
        
        # Define the Target Pose
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        # Simple conversion of Theta (Z-axis rotation) to Quaternion
        # This keeps the tool pointing straight down (standard for pick and place)
        import math
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 1.0 # Tool pointing down
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0 

        # We wrap this in a MoveIt MoveGroup goal
        # For simplicity in this step, we will use MoveIt's planning scene
        self.get_logger().info(f'Planning move to X:{x} Y:{y} Z:{z}...')
        
        # Note: In a full implementation, you'd fill out goal_msg.request.goal_constraints
        # For now, let's confirm your MoveIt 2 node is reachable.
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class UR12ePositionOnlyTest(Node):
    def __init__(self):
        super().__init__('ur12e_pos_test')
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Starting Position-Only Test...")

    def move_to_xyz(self, x, y, z):
        self.get_logger().info(f"Targeting X:{x} Y:{y} Z:{z} (Ignoring Orientation)")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_arm"
        
        # Define the target position
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        # Use a Position Constraint ONLY (No Orientation)
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.02]))
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pc)
        
        goal_msg.request.goal_constraints.append(c)
        goal_msg.request.allowed_planning_time = 10.0

        self.move_group_client.wait_for_server()
        self.move_group_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = UR12ePositionOnlyTest()
    # Use a safer, closer Y to start
    node.move_to_xyz(-0.05, 0.6, 0.5)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
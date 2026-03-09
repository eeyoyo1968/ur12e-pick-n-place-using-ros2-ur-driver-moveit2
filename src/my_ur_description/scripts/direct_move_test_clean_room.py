import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import PoseStamped
import time

class UR12eClearWorldTest(Node):
    def __init__(self):
        super().__init__('ur12e_clear_test')
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Publisher to clear the planning scene
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        self.get_logger().info("Clearing all virtual obstacles and testing move...")

    def clear_all_objects(self):
        # Create a message to delete all objects in the MoveIt world
        scene = PlanningScene()
        scene.is_diff = True
        obj = CollisionObject()
        obj.operation = CollisionObject.REMOVE
        # This is a bit of a hack to tell MoveIt to clear the world
        scene.world.collision_objects.append(obj)
        self.scene_pub.publish(scene)
        self.get_logger().info("Virtual obstacles cleared.")
        time.sleep(2.0)

    def move_to_xyz(self, x, y, z):
        self.get_logger().info(f"Targeting X:{x} Y:{y} Z:{z}")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_arm"
        
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        # Fixed neutral orientation (pointing down)
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 1.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        from moveit_msgs.msg import Constraints, PositionConstraint
        from shape_msgs.msg import SolidPrimitive
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.05]))
        pc.constraint_region.primitive_poses.append(target.pose)
        c.position_constraints.append(pc)
        goal_msg.request.goal_constraints.append(c)

        self.move_group_client.wait_for_server()
        self.move_group_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = UR12eClearWorldTest()
    node.clear_all_objects() # REMOVE THE WALLS
    node.move_to_xyz(0.0, 0.5, 0.5) # Move to a very safe, central spot
    rclpy.spin(node)

if __name__ == '__main__':
    main()
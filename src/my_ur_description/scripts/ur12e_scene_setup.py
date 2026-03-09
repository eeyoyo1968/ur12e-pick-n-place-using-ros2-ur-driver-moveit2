import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class UR12eScene(Node):
    def __init__(self):
        super().__init__('ur12e_scene_setup')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.timer = self.create_timer(2.0, self.publish_scene) # Publish periodically to ensure MoveIt catches it

    def publish_scene(self):
        # --- Define the Table ---
        table = CollisionObject()
        table.header.frame_id = "base_link"
        table.id = "work_table"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.05] # 2m x 2m table, 5cm thick
        
        table_pose = Pose()
        table_pose.position.x = 0.0
        table_pose.position.y = 0.0
        table_pose.position.z = -0.03 # 3cm below the robot base
        
        table.primitives.append(box)
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD
        
        # --- Define a Safety Wall (Behind Robot) ---
        wall = CollisionObject()
        wall.header.frame_id = "base_link"
        wall.id = "back_wall"
        
        wall_box = SolidPrimitive()
        wall_box.type = SolidPrimitive.BOX
        wall_box.dimensions = [0.1, 2.0, 2.0] # Thin wall
        
        wall_pose = Pose()
        wall_pose.position.x = -0.4 # 40cm behind the base
        wall_pose.position.z = 1.0
        
        wall.primitives.append(wall_box)
        wall.primitive_poses.append(wall_pose)
        wall.operation = CollisionObject.ADD

        self.publisher.publish(table)
        self.publisher.publish(wall)
        self.get_logger().info("Published safety table and walls to MoveIt.")

def main():
    rclpy.init()
    node = UR12eScene()
    rclpy.spin(node)
    rclpy.shutdown()
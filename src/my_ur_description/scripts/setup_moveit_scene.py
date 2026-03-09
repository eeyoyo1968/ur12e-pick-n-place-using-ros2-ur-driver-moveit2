import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class MoveIt2SceneManager(Node):
    def __init__(self):
        super().__init__('moveit_scene_manager')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        # Give the publisher a second to connect
        time.sleep(2.0)

    def make_box(self, name, x, y, z, size_x, size_y, size_z):
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = name
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def setup_scene(self):
        # 1. Table
        table = self.make_box("table", 0.5, 0.0, -0.05, 2.0, 2.0, 0.1)
        self.publisher.publish(table)
        
        # 2. Back Wall
        wall = self.make_box("back_wall", -0.3, 0.0, 0.5, 0.001, 2.0, 1.0)
        self.publisher.publish(wall)
        
        # 3. Virtual Bins (For your step-by-step test)
        soft_bin = self.make_box("soft_bin", 0.3, -0.4, 0.1, 0.3, 0.3, 0.2)
        hard_bin = self.make_box("hard_bin", 0.3, 0.4, 0.1, 0.3, 0.3, 0.2)
        self.publisher.publish(soft_bin)
        self.publisher.publish(hard_bin)

        self.get_logger().info("Scene objects published to MoveIt 2.")

import time
def main():
    rclpy.init()
    node = MoveIt2SceneManager()
    node.setup_scene()
    # Keep node alive briefly to ensure message delivery
    time.sleep(1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
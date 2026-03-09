import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
import time

class UR12eSceneManager(Node):
    def __init__(self):
        super().__init__('ur12e_scene_manager')
        # Publishing to /planning_scene allows us to set colors and transparency
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.get_logger().info("Initializing Transparent Scene Setup...")
        time.sleep(2.0) # Wait for MoveGroup to connect

    def create_box(self, name, x, y, z, sx, sy, sz):
        """Helper to create a CollisionObject box."""
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = name
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [sx, sy, sz]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def setup_full_scene(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True # Apply changes to the current scene

        # --- 1. DEFINE OBJECTS ---
        # Table (Large surface)
        #table = self.create_box("table", 0.5, 0.0, -0.05, 1.2, 1.5, 0.1)
        table = self.create_box("table", 0.0, 0.73, -0.035, 0.8, 1.2, 0.1)
        # Soft Bin (Left)
        soft_bin = self.create_box("soft_bin", 0.25, 0.3, 0.075, 0.2, 0.15, 0.15)
        # Hard Bin (Right)
        hard_bin = self.create_box("hard_bin", -0.25, 0.3, 0.075, 0.2, 0.15, 0.15)
        # Walls
        front_wall = self.create_box("front_wall", -0.7, 0.25, 0.5, 0.001, 2.5, 1.0)
        back_wall = self.create_box("back_wall", 0.6, 0.25, 0.5, 0.001, 2.5, 1.0)
        right_wall = self.create_box("right_wall", 0.0, -1.0, 0.5, 1.5, 0.001, 1.0)

        scene_msg.world.collision_objects.extend([table, soft_bin, hard_bin, back_wall, front_wall, right_wall])

        # --- 2. DEFINE COLORS & TRANSPARENCY (Alpha 'a') ---
        # Colors: Grey for table, Blue/Red for bins, Cyan for walls
        colors = [
            ("table", ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8)),     # Semi-solid table
            ("soft_bin", ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.6)),  # Greenish
            ("hard_bin", ColorRGBA(r=0.8, g=0.0, b=0.0, a=0.6)),  # Reddish
            ("back_wall", ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.3)), # Very transparent
            ("front_wall", ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.3)), # Very transparent
            ("right_wall", ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.3)) # Very transparent
        ]

        for obj_id, color_rgba in colors:
            oc = ObjectColor()
            oc.id = obj_id
            oc.color = color_rgba
            scene_msg.object_colors.append(oc)

        # Publish
        self.scene_pub.publish(scene_msg)
        self.get_logger().info("Full transparent scene published to MoveIt 2.")

def main():
    rclpy.init()
    node = UR12eSceneManager()
    node.setup_full_scene()
    time.sleep(1.0) # Ensure message is sent
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
import math
import time

from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject

class ClearAndMove(Node):
    def __init__(self):
        super().__init__('clear_and_move')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.joint_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        # Publisher to modify the planning scene
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
    def clear_scene(self):
        self.get_logger().info("Wiping Planning Scene...")
        msg = PlanningScene()
        msg.is_diff = True
        # Object with REMOVE operation
        obj = CollisionObject()
        obj.operation = CollisionObject.REMOVE
        msg.robot_state.is_diff = True
        msg.world.collision_objects.append(obj)
        self.scene_pub.publish(msg)
        time.sleep(1.0) # Wait for MoveIt to process the wipe

    def test_move(self, x, y, z):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'ur_arm'
        request.ik_request.avoid_collisions = False # Now we want to see if it's clear
        
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation.w = 1.0 # Pointing down
        
        request.ik_request.pose_stamped = target
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = ClearAndMove()
    
    # 1. Clear the "ghost" walls
    node.clear_scene()
    
    # 2. Try the move again
    res = node.test_move(0.3, 0.3, 0.6)
    
    if res.error_code.val == 1:
        node.get_logger().info("SUCCESS: Collision was the problem. Robot is now free to move.")
        # ... (Insert publishing logic here)
    else:
        node.get_logger().error(f"STILL FAILING: Error {res.error_code.val}. Point might be out of reach.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
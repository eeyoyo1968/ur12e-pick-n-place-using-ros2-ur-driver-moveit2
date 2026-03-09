import rclpy
from rclpy.node import Node
import math
import time

# Corrected Imports
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

class IKToJMove(Node):
    def __init__(self):
        super().__init__('ik_to_jmove')
        # Service to calculate the math (IK)
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # Publisher that we know works for your bins
        self.joint_pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.get_logger().info("IK-to-JMove Test Node Online.")

    def move_to_xyz_via_ik(self, x, y, z, theta_deg=90.0):
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt IK service...')

        # 1. Setup IK Request
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'ur_arm'
        request.ik_request.avoid_collisions = False # Let's ignore walls for this test
        
        theta_rad = math.radians(theta_deg)
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        
        # Tool pointing down + rotation
        target.pose.orientation.x = 1.0 * math.sin(theta_rad/2)
        target.pose.orientation.y = 1.0 * math.cos(theta_rad/2)
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0
        
        request.ik_request.pose_stamped = target

        self.get_logger().info(f"Requesting IK for X:{x} Y:{y} Z:{z}...")
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        
        # 2. If MoveIt finds a solution, send it to the Joint Controller
        if response.error_code.val == 1: # 1 = SUCCESS
            joint_positions = response.solution.joint_state.position
            # Filter only the 6 UR joints (ignoring gripper joints if present)
            # Adjust joint names if your robot has a different order
            self.get_logger().info("IK SUCCESS! Sending joints to controller...")
            
            msg = JointTrajectory()
            msg.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            
            point = JointTrajectoryPoint()
            # We take the first 6 joints from the IK solution
            point.positions = list(joint_positions[:6])
            point.time_from_start.sec = 4
            msg.points.append(point)
            
            self.joint_pub.publish(msg)
            self.get_logger().info("Trajectory published to /scaled_joint_trajectory_controller")
        else:
            self.get_logger().error(f"IK Failed. Error Code: {response.error_code.val}")
            self.get_logger().warn("Hint: -31 means 'No Solution Found'. Check if point is in reach.")

def main():
    rclpy.init()
    node = IKToJMove()
    # Testing your specific target
    node.move_to_xyz_via_ik(-0.051, 0.922, 0.502, 90.0)
    # Give it a second to send before shutting down
    time.sleep(1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
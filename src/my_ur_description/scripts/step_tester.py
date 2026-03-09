import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class UR12eStepTester(Node):
    def __init__(self):
        super().__init__('ur12e_step_tester')
        
        self.joint_pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Mapped to your setup_moveit_scene2.py coordinates
        self.bin_locations = {
            "soft": {"x": 0.25, "y": 0.3, "z": 0.25},  # Placed slightly above bin z=0.075
            "hard": {"x": -0.25, "y": 0.3, "z": 0.25}
        }
        
        # Your specific Home Point
        self.home_point = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]

    def quat_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def move_joint(self, positions, duration=5.0):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        msg.points.append(point)
        self.joint_pub.publish(msg)
        time.sleep(duration + 0.5)

    def execute_test_cycle(self, obj_class, obj_pose):
        """Step-by-step validation in URSim"""
        yaw = self.quat_to_yaw(obj_pose.orientation)
        x, y, z = obj_pose.position.x, obj_pose.position.y, obj_pose.position.z
        
        self.get_logger().info(f"--- STARTING TEST: {obj_class} ---")

        # 1. Start at Home
        self.get_logger().info("Moving to Home...")
        self.move_joint(self.home_point)

        # 2. Approach Object (Safe Z)
        self.get_logger().info(f"Approaching object at X:{x} Y:{y}...")
        # Note: Use your MoveIt cartesian_mover.move_to_xyz_theta here
        time.sleep(3.0)

        # 3. Pick Action
        self.get_logger().info("Action: Lowering and Grasping...")
        time.sleep(2.0)

        # 4. Sorting to Bin
        bin_type = "soft" if "soft" in obj_class.lower() else "hard"
        target = self.bin_locations[bin_type]
        self.get_logger().info(f"Sorting to {bin_type.upper()} bin at {target['x']}, {target['y']}...")
        # Note: Use your MoveIt cartesian_mover.move_to_xyz here
        time.sleep(4.0)

        # 5. Finish
        self.get_logger().info("Releasing and returning Home...")
        self.move_joint(self.home_point)
        self.get_logger().info("--- TEST COMPLETE ---")

def main():
    rclpy.init()
    tester = UR12eStepTester()
    
    # Example Perception Input
    test_pose = Pose()
    test_pose.position.x = 0.0
    test_pose.position.y = 0.6  # Centered on your new table
    test_pose.position.z = 0.1
    test_pose.orientation.w = 1.0
    
    tester.execute_test_cycle("soft_toy", test_pose)
    rclpy.shutdown()
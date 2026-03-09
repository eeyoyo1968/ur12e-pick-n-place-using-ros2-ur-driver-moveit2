import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Note: Using the Cartesian logic from your UR12eCartesianMover
class UR12eStepTester(Node):
    def __init__(self):
        super().__init__('ur12e_step_tester')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        
        # Bin Locations (Adjust these XYZ based on your URSim layout)
        self.bins = {
            "soft": {"x": 0.25, "y": 0.3, "z": 0.2},
            "hard": {"x": -0.25, "y": 0.3, "z": 0.2}
        }
        
        # Your specific Home Point
        self.home_point = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]

    def quat_to_yaw(self, q):
        """Converts quaternion to yaw (theta_z)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def move_joint(self, positions, duration=5.0):
        """Standard Joint move for Home and Bins."""
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        msg.points.append(point)
        self.joint_pub.publish(msg)
        time.sleep(duration + 0.5)

    def execute_cycle(self, obj_class, obj_pose):
        """The step-by-step logic you requested."""
        
        # 1. Parse Data
        yaw = self.quat_to_yaw(obj_pose.orientation)
        x, y, z = obj_pose.position.x, obj_pose.position.y, obj_pose.position.z
        
        self.get_logger().info(f"--- Starting Cycle for Class: {obj_class} ---")

        # 2. Move to position ABOVE object
        self.get_logger().info("Step 1: Moving to Approach Position (z+0.1)...")
        # In a real test, call your move_to_xyz_theta function here
        # self.move_to_xyz_theta(x, y, z + 0.1, yaw) 
        time.sleep(4.0)

        # 3. Move DOWN to grasp
        self.get_logger().info("Step 2: Descending to Grasp...")
        # self.move_to_xyz_theta(x, y, z, yaw)
        time.sleep(2.0)

        # 4. Simulate Grasp
        self.get_logger().info("Step 3: Grasping...")
        time.sleep(1.0)

        # 5. Move to Bin based on Class
        bin_type = "soft" if "soft" in obj_class.lower() else "hard"
        target_bin = self.bins[bin_type]
        self.get_logger().info(f"Step 4: Moving to {bin_type.upper()} BIN...")
        # self.move_to_xyz(target_bin['x'], target_bin['y'], target_bin['z'])
        time.sleep(4.0)

        # 6. Release and GO HOME
        self.get_logger().info("Step 5: Releasing and Returning Home...")
        self.move_joint(self.home_point)

        self.get_logger().info("--- Cycle Complete. Exiting. ---")

def main():
    rclpy.init()
    tester = UR12eStepTester()
    
    # FOR TESTING AT HOME: We simulate receiving one message from perception
    # In reality, you'd wait for a subscription here.
    fake_pose = Pose()
    fake_pose.position.x = 0.4
    fake_pose.position.y = 0.1
    fake_pose.position.z = 0.15
    fake_pose.orientation.w = 1.0 # 0 degrees yaw
    
    tester.execute_cycle("soft_toy", fake_pose)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
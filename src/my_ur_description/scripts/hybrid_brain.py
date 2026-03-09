import rclpy
from rclpy.node import Node
import math
import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HybridVisionBrain(Node):
    def __init__(self):
        super().__init__('hybrid_vision_brain')
        
        # 1. IK Service Client (The Calculator)
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # 2. Joint Publisher (The Mover - known to work)
        self.joint_pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # 3. Perception Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.class_sub = self.create_subscription(String, '/grasp/class', self.class_cb, 10)
        
        self.is_busy = False
        self.current_class = ""
        
        # Your specific Home Position
        self.home_joints = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0] 
        self.bin_soft = [4.7123, -0.9472, -2.5353, -1.2301, 1.5707, 0.0]

        self.get_logger().info("HYBRID BRAIN READY: Using IK + Direct JMove")

    def jmove(self, joints, label="Move", duration=4.0):
        self.get_logger().info(f"ACTION: {label}")
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        point.time_from_start.sec = int(duration)
        msg.points.append(point)
        self.joint_pub.publish(msg)
        time.sleep(duration + 0.5)

    def get_ik_joints(self, x, y, z, theta_deg):
        """Asks MoveIt for the joint solution without moving the robot."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_arm'
        req.ik_request.avoid_collisions = False # Bypass the Error -15 issues
        
        theta_rad = math.radians(theta_deg)
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        
        # Orientation: Pointing Down
        target.pose.orientation.x = 1.0 * math.sin(theta_rad/2)
        target.pose.orientation.y = 1.0 * math.cos(theta_rad/2)
        target.pose.orientation.w = 0.0
        
        req.ik_request.pose_stamped = target
        
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res.error_code.val == 1:
            return res.solution.joint_state.position[:6]
        return None

    def pose_cb(self, msg):
        if not self.is_busy and self.current_class:
            self.is_busy = True
            self.process_pick(msg.pose)
            self.is_busy = False

    def class_cb(self, msg):
        self.current_class = msg.data.strip().lower()

    def process_pick(self, pose):
        # Your coordinate math
        x_target = -0.03 - pose.position.x
        y_target = 1.28 - pose.position.y
        z_target = 0.35 # Fixed safe height for test
        
        self.get_logger().info(f"--- STARTING CYCLE: {self.current_class} ---")
        
        # Step 1: Go Home
        self.jmove(self.home_joints, "Home")

        # Step 2: Get IK for Approach
        joints = self.get_ik_joints(x_target, y_target, z_target + 0.1, 90.0)
        if joints:
            self.jmove(joints, "Approach")
            
            # Step 3: Get IK for Pick
            joints_pick = self.get_ik_joints(x_target, y_target, z_target, 90.0)
            if joints_pick:
                self.jmove(joints_pick, "Grasp", duration=2.0)
                # (Add Gripper Close Here)
                
                # Step 4: Lift
                self.jmove(joints, "Lift", duration=2.0)
                
                # Step 5: Bin
                if "glove" in self.current_class:
                    self.jmove(self.bin_soft, "Dropping in SOFT BIN")
        else:
            self.get_logger().error("IK failed for this object position.")

def main():
    rclpy.init()
    node = HybridVisionBrain()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
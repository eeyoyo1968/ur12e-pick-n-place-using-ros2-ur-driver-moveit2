import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveIt2VisionBrain(Node):
    def __init__(self):
        super().__init__('moveit2_vision_brain')
        
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.joint_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        
        self.pose_sub = self.create_subscription(PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.class_sub = self.create_subscription(String, '/grasp/class', self.class_cb, 10)
        
        self.is_busy = False
        self.current_class = ""
        self.latest_pose = None

        # Calibration
        self.home_joints = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0] 
        self.bin_soft = [4.7123, -0.9472, -2.5353, -1.2301, 1.5707, 0.0]
        self.bin_hard_above = [6.1086, -1.5707, -2.1232, -0.9798, 1.5707, 1.405]
        self.bin_hard = [6.1086, -1.7453, -2.2340, -0.7243, 1.5707, 1.4137]

        self.get_logger().info("BRAIN READY: Waiting for Perception...")

    def pose_cb(self, msg):
        if not self.is_busy and self.current_class:
            self.latest_pose = msg.pose
            self.process_cycle()

    def class_cb(self, msg):
        self.current_class = msg.data.strip().lower()

    def jmove(self, joints, label="Joint Move"):
        self.get_logger().info(f"ACTION: {label}")
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 4
        msg.points.append(point)
        self.joint_pub.publish(msg)
        time.sleep(5.0)

    def move_to_cartesian(self, x, y, z, theta_rad, label="Move"):
        self.get_logger().info(f"PLANNING {label}: X:{x:.3f} Y:{y:.3f} Z:{z:.3f}")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_arm"
        
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        
        # Tool Downward Orientation
        target.pose.orientation.x = 1.0 * math.sin(theta_rad/2)
        target.pose.orientation.y = 1.0 * math.cos(theta_rad/2)
        target.pose.orientation.w = 0.0

        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))
        pc.constraint_region.primitive_poses.append(target.pose)
        c.position_constraints.append(pc)
        goal_msg.request.goal_constraints.append(c)

        # CRITICAL: Wait for server and result
        self.move_group_client.wait_for_server()
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        
        # We wait 4 seconds for MoveIt to actually execute the trajectory in URSim
        time.sleep(4.5) 
        return True

    def process_cycle(self):
        self.is_busy = True
        p = self.latest_pose.position
        
        # 1. Transform math (Double check these offsets vs your table!)
        # Original: xbase = -0.03 - p.x | ybase = 1.28 - p.y
        x_t = -0.03 - p.x
        y_t = 1.28 - p.y
        z_t = max(0.232, 0.20 - 0.02 + p.z) # Safety Floor
        
        q = self.latest_pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        theta = -yaw + math.pi/2

        try:
            self.get_logger().info(f"--- STARTING PICK: {self.current_class} ---")
            
            # Step 1: Start from a clean Home
            self.jmove(self.home_joints, "Go Home")
            
            # Step 2: Move to Object (Approach)
            # Increase approach to 20cm to see the motion clearly
            self.move_to_cartesian(x_t, y_t, z_t + 0.2, theta, "Approach")
            
            # Step 3: Move to Pick
            self.move_to_cartesian(x_t, y_t, z_t, theta, "Grasp")
            time.sleep(2.0)
            
            # Step 4: Lift
            self.move_to_cartesian(x_t, y_t, z_t + 0.2, theta, "Lift")

            # Step 5: Binning
            if "glove" in self.current_class:
                self.jmove(self.bin_soft, "Sort Soft")
            else:
                self.jmove(self.bin_hard_above, "Sort Hard")
                
            self.jmove(self.home_joints, "Final Home")

        finally:
            self.is_busy = False
            self.latest_pose = None
            self.current_class = ""
            self.get_logger().info("--- CYCLE COMPLETE ---")

def main():
    rclpy.init()
    node = MoveIt2VisionBrain()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
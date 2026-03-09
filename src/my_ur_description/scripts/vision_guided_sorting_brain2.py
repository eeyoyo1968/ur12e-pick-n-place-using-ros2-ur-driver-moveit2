import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import time
import collections
import numpy as np
import math
import minimalmodbus 
from action_msgs.msg import GoalStatus

# Import your Extended Planner class
from jmove_plan_sync_async import UR12eHybridPlanner as UR12eExtendedPlanner

class VisionSortingBrain(UR12eExtendedPlanner):
    def __init__(self):
        super().__init__()
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.latest_pose = None
        self.gripper_connected = False

        self.GRIPPER_OPEN = 3
        self.GRIPPER_CLOSED = 228 



        # 1. Initialize Table Collision Object
        self.add_table()

        # 2. Gripper Initialization (Safe Try/Except)
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.2
            self.gripper.write_register(1000, 0x0100) # Activation
            self.gripper_connected = True
            self.get_logger().info("Robotiq Gripper Connected.")
        except Exception as e:
            self.get_logger().error(f"Gripper Connection Failed: {e}")

        # 3. Vision Subscriptions
        self.obj_pose_sub = self.create_subscription(PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.obj_class_sub = self.create_subscription(String, '/grasp/class', self.class_cb, 10)


    def pose_cb(self, msg):
        """Callback for the object pose from vision."""
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        """Callback for the object classification (e.g., 'glove')."""
        if not self.is_busy:
            self.current_class = msg.data

    def add_table(self):
        """Adds a virtual table to MoveIt to prevent ground collisions."""
        # This setup assumes the table surface is slightly below base Z=0
        table_pose = Pose()
        table_pose.position.x = 0.0
        table_pose.position.y = 0.6  # Centered in front of robot
        table_pose.position.z = -0.02 # Surface just below robot base
        
        # In a real script, you would publish to /collision_object 
        # For now, ensure your MoveIt config includes this or call your existing add_table()
        self.get_logger().info("Virtual Table added to Planning Scene.")

    def set_gripper(self, position):
        if self.gripper_connected:
            try:
                # rACT=1, rGTO=1, rPR=position, rSP=255, rFR=150
                self.gripper.write_registers(1000, [0x0900, position, 0x6432])
            except Exception:
                pass

    def check_stability(self):
        if self.latest_pose is None: return False
        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        #if len(self.pose_buffer) < 10: return False
        if len(self.pose_buffer) < 1: return False
      
        # Calculate variance to ensure object is stationary
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.015) # 1.5cm threshold

    def execute_sorting_cycle(self):
        self.is_busy = True
        p = self.latest_pose
        
        # Coordinate math from your reference
        x_target = -0.03 - p.position.x
        y_target = 1.28 - p.position.y
        z_pick = max(0.232, 0.20 - 0.02 + p.position.z)
        
        # Orientation (Theta)
        q = p.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        angle_rad = -theta + (math.pi/2)

        self.get_logger().info(f"Target Acquired: {self.current_class}. Moving...")

        try:
            # Step-by-step Planned Motion
            self.set_gripper(self.GRIPPER_OPEN)
            
            # Approach 10cm above
            self.move_plan_xyz_theta_sync(x_target, y_target, z_pick + 0.1, angle_rad)
            
            # Descend to Pick
            self.move_plan_xyz_theta_sync(x_target, y_target, z_pick, angle_rad)
            self.set_gripper(self.GRIPPER_CLOSED)
            time.sleep(1.0)
            
            # Lift
            self.move_plan_xyz_theta_sync(x_target, y_target, z_pick + 0.15, angle_rad)
            
            # Sort to Bins
            if self.current_class == "glove":
                bin_soft = np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0]) * (math.pi/180)
                self.jmove_plan_sync(bin_soft)
            else:
                bin_hard = np.array([350.0, -100.0, -128.0, -41.5, 90.0, 81.0]) * (math.pi/180)
                self.jmove_plan_sync(bin_hard)

            self.set_gripper(self.GRIPPER_OPEN)
            time.sleep(0.5)
            self.jmove_plan_sync(self.home_seed)

        finally:
            self.is_busy = False
            self.pose_buffer.clear()

def main(args=None):
    rclpy.init(args=args)
    node = VisionSortingBrain()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.is_busy and node.check_stability():
                node.execute_sorting_cycle()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
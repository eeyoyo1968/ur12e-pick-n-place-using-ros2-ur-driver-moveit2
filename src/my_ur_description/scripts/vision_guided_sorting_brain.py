import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 
import math
import minimalmodbus 
from action_msgs.msg import GoalStatus

# Inherit from the Extended Planner we created previously
# Ensure the file containing the Extended Planner is importable
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

        # Gripper Initialization
        print("Connecting to Gripper...")
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.1
            self.gripper.read_register(2000)
            self.gripper.write_register(1000, 0x0100)
            self.gripper_connected = True
            self.get_logger().info("Gripper Online.")
        except Exception as e:
            self.get_logger().warn(f"Gripper offline: {e}")

        # Vision Subscriptions
        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10)

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data

    def gripper_heartbeat(self):
        """Keep the Robotiq watchdog happy."""
        if self.gripper_connected:
            try: self.gripper.read_register(2000)
            except: pass

    def set_gripper(self, position):
        if self.gripper_connected:
            try: self.gripper.write_registers(1000, [0x0900, position, 0x6432])
            except: pass

    def get_theta_from_pose(self, pose):
        """Extracts yaw (theta) from quaternion."""
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def check_stability(self):
        """Ensures the vision target is not jumping."""
        if self.latest_pose is None or self.latest_pose.position.x == 0.0:
            return False
        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10: return False
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.02)

    def execute_sorting_cycle(self):
        self.is_busy = True
        p = self.latest_pose.position
        # Calculate theta based on your coordinate mapping
        theta = -self.get_theta_from_pose(self.latest_pose) + math.pi/2
        
        # Coordinate Transformation
        x_t, y_t = -0.03 - p.x, 1.28 - p.y
        z_t = max(0.232, 0.20 - 0.02 + p.z)

        # Pose Definitions
        D2R = math.pi / 180.0
        bin_soft = np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0]) * D2R
        bin_hard_above = np.array([350.0, -90.0, -121.65, -56.14, 90.0, 80.5]) * D2R
        bin_hard = np.array([350.0, -100.0, -128.0, -41.5, 90.0, 81.0]) * D2R

        try:
            self.get_logger().info(f"Sorting {self.current_class} at X:{x_t:.3f} Y:{y_t:.3f}")

            # 1. Approach (Planned Sync Move)
            self.set_gripper(self.GRIPPER_OPEN)
            self.move_plan_xyz_theta_sync(x_t, y_t, z_t + 0.10, theta)

            # 2. Pick (Planned Sync Move)
            self.move_plan_xyz_theta_sync(x_t, y_t, z_t, theta)
            self.set_gripper(self.GRIPPER_CLOSED)
            time.sleep(1.2) # Wait for grasp

            # 3. Retract (Planned Sync Move)
            self.move_plan_xyz_theta_sync(x_t, y_t, z_t + 0.15, theta)

            # 4. Binning based on class
            if self.current_class == "glove":
                self.get_logger().info("Moving to Soft Bin...")
                self.jmove_plan_sync(bin_soft)
            else:
                self.get_logger().info("Moving to Hard Bin...")
                self.jmove_plan_sync(bin_hard_above)
                # Use direct jmove_sync for the final drop if close to the bin edge
                self.jmove_sync(bin_hard, duration=3.0)

            # 5. Release and Return Home
            self.set_gripper(self.GRIPPER_OPEN)
            time.sleep(1.0)
            self.jmove_plan_sync(self.home_seed)

        except Exception as e:
            self.get_logger().error(f"Cycle failed: {e}")
        finally:
            self.is_busy = False
            self.pose_buffer.clear()
            self.get_logger().info("Cycle Finished. Ready for next object.")

def main(args=None):
    rclpy.init(args=args)
    brain = VisionSortingBrain()

    print("VISION BRAIN READY.")
    try:
        while rclpy.ok():
            rclpy.spin_once(brain, timeout_sec=0.1)
            brain.gripper_heartbeat()
            if not brain.is_busy and brain.check_stability():
                brain.execute_sorting_cycle()
    except KeyboardInterrupt:
        pass
    finally:
        brain.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
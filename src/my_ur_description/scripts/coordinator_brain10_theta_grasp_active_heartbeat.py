import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 
import math
import minimalmodbus 

from rclpy.callback_groups import ReentrantCallbackGroup
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        self.group = ReentrantCallbackGroup()
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.target_class = "unknown"
        self.latest_pose = None
        self.gripper_connected = False

        self.GRIPPER_OPEN = 3
        self.GRIPPER_CLOSED = 228 

        print("Connecting to Gripper...")
        for i in range(3): # Reduced retries for speed
            try:
                self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
                self.gripper.serial.baudrate = 115200
                self.gripper.serial.timeout = 0.1 # Faster timeout
                self.gripper.read_register(2000)
                self.gripper.write_register(1000, 0x0100)
                self.gripper_connected = True
                self.get_logger().info("Gripper Online.")
                break
            except Exception:
                time.sleep(0.5)

        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10, callback_group=self.group)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10, callback_group=self.group)

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data
            self.target_class = msg.data

    def gripper_heartbeat(self):
        """Mandatory ping to keep LED Blue"""
        if not self.gripper_connected: return
        try:
            self.gripper.read_register(2000)
        except:
            pass

    def active_wait(self, seconds):
        """Wait while keeping the gripper watchdog happy"""
        start = time.time()
        while (time.time() - start) < seconds:
            self.gripper_heartbeat()
            time.sleep(0.05) # High frequency heartbeat

    def set_gripper(self, position):
        if not self.gripper_connected: return
        try:
            self.gripper.write_registers(1000, [0x0900, position, 0x6432])
        except:
            pass

    def get_theta_from_pose(self, pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def check_stability(self):
        # FIXED: Corrected syntax error from previous version
        if self.latest_pose is None or self.latest_pose.position.x == 0.0:
            return False
        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10: return False
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.02)

    def execute_move(self):
        self.is_busy = True
        p = self.latest_pose.position
        theta = -self.get_theta_from_pose(self.latest_pose)+math.pi/2
        
        # Fast trajectory coordinates
        x_t, y_t = -0.03 - p.x, 1.28 - p.y
        z_t = max(0.232, 0.20 - 0.02 + p.z)

        self.get_logger().info(f"STARTING FAST PICK: {self.target_class}")
        
            # --- Math Constants ---
        PI = math.pi
        D2R = PI / 180.0
        #bin_hard=np.array([330.0, -65.72, -140.5, -63.9, 90.0, 60.0])*D2R
        bin_soft=np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0])*D2R  

        bin_hard=np.array([350.0, -100.0, -128.0, -41.5, 90.0, 81.0])*D2R   
        bin_hard_above=np.array([350.0, -90.0, -121.65, -56.14, 90.0, 80.5])*D2R 
        
        try:
            # 1. Approach
            self.set_gripper(self.GRIPPER_OPEN)
            self.move_xyz_theta_no_flip(x_t, y_t, z_t + 0.10, theta)
            #self.active_wait(0.8) # Reduced wait
            
            # 2. Pick
            self.move_xyz_theta_no_flip(x_t, y_t, z_t, theta)
            #self.active_wait(0.7) 
            self.set_gripper(self.GRIPPER_CLOSED)
            self.active_wait(1.0) # Grasp time
            
            # 3. Binning
            self.move_xyz_theta_no_flip(x_t, y_t, z_t + 0.15, theta)
            #self.active_wait(0.5)
            
            #bin_x = 0.25 if self.current_class == "glove" else -0.25
            #self.move_xyz_theta_no_flip(bin_x, 0.45, 0.35, 0.0)
            #self.active_wait(1.5) 
            
            if self.current_class == "glove" : # soft bin
                self.jmove(bin_soft)
            else : # hard bin
                self.jmove(bin_hard_above)
                self.jmove(bin_hard)    
                
            # 4. Release and Reset
            self.set_gripper(self.GRIPPER_OPEN)
            #self.active_wait(0.5)
            self.active_wait(2.0)
            self.move_xyz_theta_no_flip(0.0, 0.6, 0.5, 0.0)
            #self.active_wait(0.8)

            
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")
        finally:
            print("Cycle Finished.")
            os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    # REDUCED: Startup wait reduced to 2.5s with active heartbeats
    if brain.gripper_connected:
        print("Fast Activation...")
        #brain.active_wait(2.5)
        #brain.active_wait(0.1)

    print("READY.")
    while rclpy.ok():
        rclpy.spin_once(brain, timeout_sec=0.01)
        brain.gripper_heartbeat()
        if brain.check_stability():
            brain.execute_move()

if __name__ == '__main__':
    main()
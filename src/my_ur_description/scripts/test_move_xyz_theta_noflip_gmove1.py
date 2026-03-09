import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math
import numpy as np

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from moveit_msgs.msg import AttachedCollisionObject

# Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState 

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped # <--- Corrected Import
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

from std_msgs.msg import String

class UR12eController(Node):
    def __init__(self):
        super().__init__('ur12e_controller')
        
        # Action Clients
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        #self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')

        # Feedback variables
        self.current_gripper_pos = 0.0
        
        # Subscriber to Joint States
        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Add a publisher for the URScript interface
        #self.script_pub = self.create_publisher(String, '/ur_script_interface/script_command', 10)
        # Update this line in your __init__:
        self.script_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)

        self.get_logger().info("Connecting to controllers...")
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info("System Online.")

    #def call_gripper(self, state):
    #    req = SetBool.Request()
    #    req.data = state # True for Close, False for Open
    #    self.gripper_client.call_async(req)
    #    self.get_logger().info('Sent gripper command to service...')

    def joint_state_callback(self, msg):
        """Updates the gripper knuckle joint position in real-time."""
        self.last_joint_msg = msg # Store this for start_state

        joint_name = 'robotiq_85_left_knuckle_joint'
        if joint_name in msg.name:
            idx = msg.name.index(joint_name)
            self.current_gripper_pos = msg.position[idx]
    def gripper_move_wrist(self, pos_0_to_255):
        """
        Controls the 2F-85 through the UR wrist pins.
        pos_0_to_255: 0 is fully open, 255 is fully closed.
        """
        # This script assumes you have initialized tool communication
        # and that the Robotiq URCap logic is available to the controller.
        script_msg = String()
        script_msg.data = f"rq_move_and_wait({int(pos_0_to_255)})\n"
        self.script_pub.publish(script_msg)
        self.get_logger().info(f"Sent wrist-gripper command: {pos_0_to_255}")
        time.sleep(1.5) # Wait for hardware execution

    def gripper_move(self, pos):
        """Moves the gripper and waits for the physical hardware result."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(pos)]
        # In real hardware, give it time to move
        point.time_from_start.sec = 2 
        goal_msg.trajectory.points.append(point)
        
        self.get_logger().info(f"Gripper moving to {pos}...")
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        
        # Wait for the server to acknowledge the goal
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        # Wait for the physical movement to finish
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info("Gripper motion complete.")
        return True

    def check_grasp_success(self):
        """
        Logic: If position > 0.75, the gripper is empty.
        In simulation, this is highly accurate.
        """
        self.get_logger().info(f"Grasp Check - Position: {self.current_gripper_pos:.4f}")
        return self.current_gripper_pos < 0.75

    def jmove(self, jointvector):
        """Moves the UR12e Arm to a joint configuration"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # FIX: Populate the start state with your actual current positions 
        # from the joint_state_callback
        if hasattr(self, 'last_joint_msg'):
            goal_msg.request.start_state.joint_state = self.last_joint_msg
        else:
            goal_msg.request.start_state.is_diff = True 
        
        goal_msg.request.allowed_planning_time = 5.0
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(joint_names, jointvector):
            jc = JointConstraint(joint_name=name, position=float(pos), 
                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        goal_msg.request.max_velocity_scaling_factor = 0.1

        future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def jmove_async(self, jointvector):
        """Starts a joint move and returns the future immediately."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # Populate start state to avoid the Joint 0 'jump' error
        if hasattr(self, 'last_joint_msg'):
            goal_msg.request.start_state.joint_state = self.last_joint_msg
        else:
            goal_msg.request.start_state.is_diff = True 

        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(joint_names, jointvector):
            jc = JointConstraint(joint_name=name, position=float(pos), 
                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        goal_msg.request.max_velocity_scaling_factor = 0.1
        
        # Return the future - DO NOT spin_until_complete here
        return self._arm_client.send_goal_async(goal_msg)

    def stop_motion(self):
        """Immediately stops all UR arm motion."""
        msg = String()
        msg.data = "stopl(1.5)\n"
        self.script_pub.publish(msg)
        self.get_logger().info("Sent stopl command to UR controller.")



    # *** move to a pose ***
    def move_pose(self, x, y, z, ox, oy, oz, ow, frame_id="base_link"):
        """
        Moves the end-effector relative to frame_id (default 'base_link').
        """
        self.get_logger().info(f"Moving to XYZ: [{x}, {y}, {z}] relative to {frame_id}")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # 1. Setup Target Pose
        target_pose = Pose()
        target_pose.position = Point(x=float(x), y=float(y), z=float(z))
        target_pose.orientation = Quaternion(x=float(ox), y=float(oy), z=float(oz), w=float(ow))
        
        # 2. Build Constraints
        con = Constraints()
        
        pc = PositionConstraint()
        pc.header.frame_id = frame_id # This determines what (0,0,0) means!
        pc.link_name = "tool0"
        
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.005] # 5mm tolerance for easier planning
        
        pc.constraint_region.primitives.append(sp)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0
        con.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = "tool0"
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        con.orientation_constraints.append(oc)

        goal_msg.request.goal_constraints.append(con)
        
        # Set planning parameters
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        
        future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def move_xyz(self, x, y, z):
        """
        Points vertical downward and moves to XYZ.
        Standard UR 'down' orientation is often [1, 0, 0, 0] 
        meaning 180 degree rotation around X.
        """
        # Adjusted Quaternion for 'Pointing Down'
        # If the robot moves weirdly, we can tune these.
        return self.move_pose(x, y, z, 1.0, 0.0, 0.0, 0.0)

    def move_xyz_base(self, x, y, z):
        """Use this if you measured from the robot's base."""
        return self.move_pose(x, y, z, 1.0, 0.0, 0.0, 0.0, frame_id="base_link")

    def move_xyz_world(self, x, y, z):
        """Use this if you measured from the floor (origin of simulation)."""
        return self.move_pose(x, y, z, 1.0, 0.0, 0.0, 0.0, frame_id="world")

    # *** move to a pose with IK and without flippings ***

    def get_ik(self, x, y, z, frame_id="base_link", seed_joints=None):
        """
        Uses MoveIt IK service. 
        If seed_joints is provided, it uses those instead of current state.
        """
        ik_client = self.create_client(GetPositionIK, 'compute_ik')
        while not ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')

        request = GetPositionIK.Request()
        ik_request = PositionIKRequest()
        ik_request.group_name = "ur_manipulator"
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        target_pose.pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        
        ik_request.pose_stamped = target_pose
        ik_request.avoid_collisions = True

        # --- SEED LOGIC ---
        if seed_joints is not None:
            ik_request.robot_state.joint_state.name = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            ik_request.robot_state.joint_state.position = [float(p) for p in seed_joints]
        # ------------------
        
        request.ik_request = ik_request
        future = ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(f"IK Error Detail: {response.error_code.val}") 
        # -31: No Solution, -1: Device Error, -10: Collision

        if response.error_code.val == 1:
            return response.solution.joint_state.position
        else:
            return None



    def get_ik_pose(self, x, y, z, qx, qy, qz, qw, frame_id="base_link", seed_joints=None):
        """
        Uses MoveIt IK service with full Pose (XYZ + Quaternion).
        """
        ik_client = self.create_client(GetPositionIK, 'compute_ik')
        while not ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')

        request = GetPositionIK.Request()
        ik_request = PositionIKRequest()
        ik_request.group_name = "ur_manipulator"
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        # Use the passed quaternion values
        target_pose.pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
        
        ik_request.pose_stamped = target_pose
        ik_request.avoid_collisions = True

        if seed_joints is not None:
            ik_request.robot_state.joint_state.name = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            ik_request.robot_state.joint_state.position = [float(p) for p in seed_joints]
        
        request.ik_request = ik_request
        future = ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val == 1:
            return response.solution.joint_state.position
        else:
            return None


    def move_xyz_no_flip(self, x, y, z, frame_id="base_link"):
        self.get_logger().info(f"Targeting Cartesian: ({x}, {y}, {z})")
        
        # We use 'home' as the seed to keep the configuration consistent (no flips)
        #home_seed = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
        #home_seed = [4.7124, -0.7854, -2.3562, -1.5707, 1.5707, 0.0]
        home_seed = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        
        joint_solution = self.get_ik(x, y, z, frame_id, seed_joints=home_seed)
        
        if joint_solution is not None:
            # Slicing for the 6 arm joints
            arm_joints = list(joint_solution[:6])
            
            # Print for debugging
            print_joints = [round(p, 3) for p in arm_joints]
            self.get_logger().info(f"Executing jmove to IK solution: {print_joints}")
            
            return self.jmove(arm_joints)
        else:
            self.get_logger().error(f"IK Failure for point ({x}, {y}, {z})")
            return False

    def move_pose_no_flip(self, x, y, z, ox, oy, oz, ow, frame_id="base_link"):
        """
        Moves to a specific Cartesian Pose (XYZ + Orientation) while 
        using a seed to prevent joint flipping.
        """
        self.get_logger().info(f"Targeting Pose (No-Flip): XYZ[{x}, {y}, {z}] ORI[{ox}, {oy}, {oz}, {ow}]")

        # 1. Define the 'Home' or 'Reference' seed to keep configuration consistent
        # This prevents the robot from choosing a flipped IK solution
        #home_seed = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
        #home_seed = [4.7124, -0.7854, -2.3562, -1.5707, 1.5707, 0.0]
        home_seed = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        # 2. Call IK service with the specific orientation
        
        ik_client = self.create_client(GetPositionIK, 'compute_ik')
        while not ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')

        request = GetPositionIK.Request()
        ik_request = PositionIKRequest()
        ik_request.group_name = "ur_manipulator"
        ik_request.avoid_collisions = True
        
        # Setup Target Pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        target_pose.pose.orientation = Quaternion(x=float(ox), y=float(oy), z=float(oz), w=float(ow))
        
        ik_request.pose_stamped = target_pose

        # Apply the Seed Configuration
        ik_request.robot_state.joint_state.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        ik_request.robot_state.joint_state.position = [float(p) for p in home_seed]
        
        request.ik_request = ik_request
        future = ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        # 3. If IK finds a solution, execute via jmove
        if response.error_code.val == 1:
            arm_joints = list(response.solution.joint_state.position[:6])
            self.get_logger().info(f"IK Success. Executing move to avoid flip.")
            return self.jmove(arm_joints)
        else:
            self.get_logger().error(f"IK Failure for Pose. Error Code: {response.error_code.val}")
            return False

    def add_table(self):
        """Adds a virtual table to the MoveIt Planning Scene."""
        self.get_logger().info("Adding virtual table to scene...")
        
        # Publisher for the planning scene
        self.scene_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        table_object = CollisionObject()
        table_object.header.frame_id = "base_link"
        table_object.id = "work_table"
        
        # Define the table shape (Box)
        # Dimensions: X=1.5m, Y=1.5m, Z=0.02m (2cm thick)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # box.dimensions = [1.5, 1.5, 0.02]
        box.dimensions = [0.26, 0.26, 0.02]
        
        
        # Define the table position
        # We place it slightly below the robot base (Z = -0.01)
        table_pose = Pose()
        table_pose.position.x = 0.0
        table_pose.position.y = -0.0
        table_pose.position.z = -0.02 # Just below the base
        
        table_object.primitives.append(box)
        table_object.primitive_poses.append(table_pose)
        table_object.operation = CollisionObject.ADD
        
        # Publish multiple times to ensure MoveIt picks it up
        for _ in range(5):
            self.scene_pub.publish(table_object)
            time.sleep(0.1)
            
        self.get_logger().info("Table added. Robot will now avoid Z < -0.01")

    def attach_gripper(self):
        """Tells MoveIt that the gripper is attached to the wrist (tool0)."""
        self.get_logger().info("Attaching gripper to tool0...")
        
        # Publisher for attached objects
        self.attached_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        
        attached_gripper = AttachedCollisionObject()
        attached_gripper.link_name = "tool0" # The link it's physically bolted to
        attached_gripper.object.header.frame_id = "tool0"
        attached_gripper.object.id = "robotiq_gripper_volume"
        
        # Approximate the gripper as a cylinder or box for safety
        # Height: 0.15m (15cm), Radius: 0.05m (5cm)
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.CYLINDER
        shape.dimensions = [0.15, 0.05] # [height, radius]
        
        # Position the 'safety volume' so it extends outward from the wrist
        pose = Pose()
        pose.position.z = 0.075 # Shift half the height so it starts at tool0
        
        attached_gripper.object.primitives.append(shape)
        attached_gripper.object.primitive_poses.append(pose)
        attached_gripper.object.operation = CollisionObject.ADD
        
        # Important: Allow the gripper to 'touch' the robot's own wrist
        attached_gripper.touch_links = ['tool0', 'wrist_3_link', 'flange']
        
        for _ in range(5):
            self.attached_pub.publish(attached_gripper)
            time.sleep(0.1)
        
        self.get_logger().info("Gripper volume attached to planning scene.")

    def move_xyz_theta(self, x, y, z, theta_rad, frame_id="base_link"):
        """
        Moves to XYZ while pointing down, rotated by theta_rad around the Z-axis.
        theta_rad = 0 means the gripper fingers are aligned with the X-axis.
        """
        self.get_logger().info(f"Moving to XYZ: ({x}, {y}, {z}) with Twist: {theta_rad:.2f} rad")

        # 1. Start with the 'Base Downward' orientation (180 deg around X)
        # 2. Apply a rotation of 'theta' around the NEW Z axis
        # Formula for combining these:
        ox = math.cos(theta_rad / 2.0)
        oy = math.sin(theta_rad / 2.0)
        oz = 0.0
        ow = 0.0
        
        # Note: In UR robots, x=1.0, y=0.0, z=0.0, w=0.0 is the standard 'down'
        # To twist that, we modify the y and z components:
        nx = 1.0 * math.cos(theta_rad / 2.0)
        ny = 1.0 * math.sin(theta_rad / 2.0)
        nz = 0.0
        nw = 0.0
        
        # Simpler approach: Use the no_flip logic with the calculated orientation
        # ox, oy, oz, ow for a 'down + twist' is:
        # q_down = [1, 0, 0, 0]
        # q_twist = [0, 0, sin(t/2), cos(t/2)]
        # Resulting Quaternion:
        qx = math.cos(theta_rad / 2.0)
        qy = -math.sin(theta_rad / 2.0)
        qz = 0.0
        qw = 0.0

        return self.move_pose_no_flip(x, y, z, qx, qy, qz, qw, frame_id)

    def move_xyz_theta_no_flip(self, x, y, z, theta_rad, frame_id="base_link"):
        """
        Moves to XYZ pointing down, with a twist theta_rad.
        Includes a guard to prevent the wrist from spinning more than 180 degrees.
        """
        self.get_logger().info(f"Targeting XYZ: ({x}, {y}, {z}) Theta: {math.degrees(theta_rad):.1f} deg")

        # 1. Calculate Quaternion for Downward + Twist
        # Standard 'pointing down' is a 180-deg rotation around X.
        # We combine that with a 'theta' rotation around the Z-axis.
        qx = math.cos(theta_rad / 2.0)
        qy = -math.sin(theta_rad / 2.0)
        qz = 0.0
        qw = 0.0

        # 2. Get IK Solution using your existing no-flip logic
        #home_seed = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
        #home_seed = [4.7124, -0.7854, -2.3562, -1.5707, 1.5707, 0.0]
        home_seed = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        
        joint_solution = self.get_ik_pose(x, y, z, qx, qy, qz, qw, frame_id, seed_joints=home_seed)

        if joint_solution is not None:
            arm_joints = list(joint_solution[:6])
            
            # 3. ROTATION GUARD: Normalize wrist_3_joint (Index 5)
            # This prevents the 'long way around' 270-degree spins.
            # It keeps the joint value between -pi and pi.
            wrist_val = arm_joints[5]
            arm_joints[5] = (wrist_val + math.pi) % (2 * math.pi) - math.pi
            
            self.get_logger().info(f"Wrist adjusted from {wrist_val:.2f} to {arm_joints[5]:.2f}")
            return self.jmove(arm_joints)
        else:
            self.get_logger().error("IK failed for this rotation.")
            return False
            
    def monitor_grasp(self, expected_min=0.1, expected_max=0.7):
        """
        Returns True if the gripper position is within a range that suggests 
        it is actually holding an object. 
        If it reaches 0.795 (fully closed), it missed the object.
        """
        # Small delay to let simulation/hardware settle
        time.sleep(0.5) 
        pos = self.current_gripper_pos
        
        if pos > 0.75:
            self.get_logger().error("Grasp Failed: Gripper is empty!")
            return False
        elif pos < 0.05:
            self.get_logger().error("Grasp Failed: Gripper did not close!")
            return False
        else:
            self.get_logger().info(f"Grasp Verified: Holding object at {pos:.3f}")
            return True
    
    def contact_search_and_retract(self, retract_dist=0.005):
        """Native URScript: Zeroes sensors, searches for table, then retracts."""
        # Use \n at the end and a clear def/end block
        script = f"""
def guarded_pick():
  zero_ftsensor()
  # direction: [x, y, z, rx, ry, rz] in Tool Frame
  # In UR12e, +Z is usually 'out' from the flange
  direction = [0, 0, 1, 0, 0, 0] 
  tool_contact(direction)
  stopl(3.0)
  
  # Retract 5mm so the gripper tips aren't dragging on the table
  p_curr = get_actual_tcp_pose()
  p_retract = pose_add(p_curr, p[0, 0, -{retract_dist}, 0, 0, 0])
  movel(p_retract, a=1.2, v=0.05)
end
\n"""
        msg = String()
        msg.data = script
        # Check your topic name! If /ur_script_interface/script_command failed, 
        # try /urscript_interface/script_command (no underscore after ur)
        self.script_pub.publish(msg)
        self.get_logger().info("Hardware Guarded Move Initiated...")


def main():
    # ... init bot ...
    rclpy.init()
    bot = UR12eController()

    # 1. Build the virtual world
    bot.add_table()
    
    # 2. Attach the gripper safety volume
    # bot.attach_gripper()

    # --- Math Constants ---
    PI = math.pi
    D2R = PI / 180.0

    # --- Pose Definitions ---
    # Home (Shoulder at -90 degrees)
    #home = [0.0, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
    #home = [3.14159, -0.7854, -2.3562, -1.5707, 1.5707, 0.0]
    home = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
    angle = -45.0 * (math.pi / 180.0) # Convert to radians

    # 1. Move to a safe "Home" using joints
    bot.jmove(home)

    # transform from the table_world_frame to the robot baselink frame
    #xbase=-0.05-(-0.188)
    xbase=-0.03-(0.021)
    #ybase=1.13+0.15+(0.375)
    ybase=1.13+0.15-(0.358)
    zbase=max(0.2275, 0.24-0.04+(0.022))

    bot.move_xyz_theta_no_flip(xbase, ybase, 0.5, angle)
    time.sleep(2)
    #bot.move_xyz_no_flip(xbase, ybase, 0.24)
    bot.move_xyz_theta_no_flip(xbase, ybase, zbase, angle)
    time.sleep(2)
    bot.move_xyz_no_flip(xbase, ybase, 0.5)
    time.sleep(2)

    # Instead of move_xyz, use the IK-guarded version
    bot.jmove(home) 


    bot.get_logger().info("Shutting down controller.")
    bot.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from action_msgs.msg import GoalStatus
import time
import numpy as np
import math

class UR12eExtendedPlanner(Node):
    def __init__(self):
        super().__init__('ur12e_extended_planner')
        
        # Action Clients
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._direct_arm_client = ActionClient(
            self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # IK Service Client
        self._ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # State tracking
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        # Reference Home for IK Seed
        self.home_seed = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        
        self.get_logger().info("UR12e Extended Planner Initialized")

    def joint_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.current_joints = [
                name_to_pos['shoulder_pan_joint'], name_to_pos['shoulder_lift_joint'],
                name_to_pos['elbow_joint'], name_to_pos['wrist_1_joint'],
                name_to_pos['wrist_2_joint'], name_to_pos['wrist_3_joint']
            ]
        except KeyError:
            pass

    # --- CORE JOINT MOVE FUNCTIONS ---

    def jmove_plan_async(self, joint_vector):
        """Planned move with collision avoidance (Async)."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0): return None
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(joint_names, joint_vector):
            jc = JointConstraint(joint_name=name, position=float(pos), 
                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        goal_msg.request.max_velocity_scaling_factor = 0.1
        return self._move_group_client.send_goal_async(goal_msg)

    def jmove_plan_sync(self, joint_vector):
        """Planned move with collision avoidance (Sync)."""
        future = self.jmove_plan_async(joint_vector)
        if not future: return False
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    # --- IK RESOLVER ---

    def get_ik_solution(self, x, y, z, qx, qy, qz, qw, frame_id="base_link"):
        """Solves IK using a reference seed to prevent flipping."""
        if not self._ik_client.wait_for_service(timeout_sec=2.0): return None
        
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = "ur_manipulator"
        ik_req.avoid_collisions = True
        
        target = PoseStamped()
        target.header.frame_id = frame_id
        target.pose.position = Point(x=float(x), y=float(y), z=float(z))
        target.pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
        
        ik_req.pose_stamped = target
        ik_req.robot_state.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                                              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        ik_req.robot_state.joint_state.position = self.home_seed
        
        req.ik_request = ik_req
        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res.error_code.val == 1:
            # Clean wrist rotation to prevent >180 spins
            joints = list(res.solution.joint_state.position[:6])
            joints[5] = (joints[5] + math.pi) % (2 * math.pi) - math.pi
            return joints
        return None

    # --- CARTESIAN PLANNED MOVES (YOUR REQUEST) ---

    def move_plan_xyz_theta_async(self, x, y, z, theta_rad, frame_id="base_link"):
        """Asynchronous planned move to XYZ + Rotation (Downwards + Twist)."""
        qx = math.cos(theta_rad / 2.0)
        qy = -math.sin(theta_rad / 2.0)
        qz, qw = 0.0, 0.0 # Standard down-orientation components
        
        joint_sol = self.get_ik_solution(x, y, z, qx, qy, qz, qw, frame_id)
        if joint_sol:
            return self.jmove_plan_async(joint_sol)
        self.get_logger().error("IK failed for move_plan_xyz_theta_async")
        return None

    def move_plan_xyz_theta_sync(self, x, y, z, theta_rad, frame_id="base_link"):
        """Synchronous planned move to XYZ + Rotation."""
        future = self.move_plan_xyz_theta_async(x, y, z, theta_rad, frame_id)
        if not future: return False
        # The logic is identical to jmove_plan_sync but starts from the async call above
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def move_plan_xyz_async(self, x, y, z, frame_id="base_link"):
        """Asynchronous planned move to XYZ (Pointing Straight Down)."""
        return self.move_plan_xyz_theta_async(x, y, z, 0.0, frame_id)

    def move_plan_xyz_sync(self, x, y, z, frame_id="base_link"):
        """Synchronous planned move to XYZ (Pointing Straight Down)."""
        return self.move_plan_xyz_theta_sync(x, y, z, 0.0, frame_id)

def main(args=None):
    rclpy.init(args=args)
    planner = UR12eExtendedPlanner()
    
    # 1. Move to Home (Planned/Sync)
    planner.jmove_plan_sync(planner.home_seed)
    
    # 2. Test Cartesian Planned Move (Sync)
    planner.get_logger().info("Executing move_plan_xyz_theta_sync...")
    planner.move_plan_xyz_theta_sync(0.2, 0.9, 0.5, math.radians(-45))
    
    # 3. Test Cartesian Planned Move (Async)
    planner.get_logger().info("Starting move_plan_xyz_async...")
    f = planner.move_plan_xyz_async(0.2, 0.4, 0.3)
    if f:
        rclpy.spin_until_future_complete(planner, f)
        planner.get_logger().info("Cartesian plan accepted and moving...")

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
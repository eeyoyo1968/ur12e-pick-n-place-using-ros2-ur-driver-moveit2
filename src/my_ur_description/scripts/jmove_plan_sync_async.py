import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
import time
import numpy as np
import math

class UR12eHybridPlanner(Node):
    def __init__(self):
        super().__init__('ur12e_hybrid_planner')
        
        # MoveIt Planning Client (Collision Avoidance)
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Direct Controller Client (No Collision Avoidance)
        self._direct_arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.get_logger().info("UR12e Hybrid Planner Initialized")

    def joint_cb(self, msg):
        """Updates internal joint state from /joint_states topic."""
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.current_joints = [
                name_to_pos['shoulder_pan_joint'],
                name_to_pos['shoulder_lift_joint'],
                name_to_pos['elbow_joint'],
                name_to_pos['wrist_1_joint'],
                name_to_pos['wrist_2_joint'],
                name_to_pos['wrist_3_joint']
            ]
        except KeyError as e:
            self.get_logger().error(f'Missing joint: {e}')

    # --- JMOVE_PLAN FUNCTIONS (MOVEIT / COLLISION AVOIDANCE) ---

    def jmove_plan_async(self, joint_vector):
        """Triggers MoveIt 2 planning. Returns future immediately."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            return None

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
        """Blocks until MoveIt motion is complete."""
        future = self.jmove_plan_async(joint_vector)
        if future is None: return False
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    # --- JMOVE FUNCTIONS (DIRECT CONTROLLER / NO COLLISION CHECK) ---

    def jmove_async(self, joint_vector, duration=5.0):
        """Directly commands controller. Returns future immediately."""
        if self.current_joints is None or not self._direct_arm_client.wait_for_server(timeout_sec=2.0):
            return None

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        p1 = JointTrajectoryPoint(positions=self.current_joints, time_from_start=rclpy.duration.Duration(seconds=0).to_msg())
        p2 = JointTrajectoryPoint(positions=[float(j) for j in joint_vector], 
                                  time_from_start=rclpy.duration.Duration(seconds=int(duration)).to_msg())

        goal_msg.trajectory.points = [p1, p2]
        return self._direct_arm_client.send_goal_async(goal_msg)

    def jmove_sync(self, joint_vector, duration=5.0):
        """Blocks until direct controller motion is complete."""
        future = self.jmove_async(joint_vector, duration)
        if future is None: return False
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

def main(args=None):
    rclpy.init(args=args)
    planner = UR12eHybridPlanner()
    
    # Wait for joint states
    start_time = time.time()
    while planner.current_joints is None and (time.time() - start_time) < 5.0:
        rclpy.spin_once(planner, timeout_sec=0.1)
    
    # --- POSE DEFINITIONS ---
    D2R = math.pi / 180.0
    home = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0] #
    bin_soft = np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0]) * D2R
    bin_hard = np.array([350.0, -100.0, -128.0, -41.5, 90.0, 81.0]) * D2R
    bin_hard_above = np.array([350.0, -90.0, -121.65, -56.14, 90.0, 80.5]) * D2R

    # --- TEST SEQUENCE ---

   
# 1. Test jmove_plan_async (MoveIt planning)
    planner.get_logger().info("Testing jmove_plan_async to Home...")
    f_home = planner.jmove_plan_async(home)
    if f_home:
        rclpy.spin_until_future_complete(planner, f_home)
        planner.get_logger().info("MoveIt accepted the plan.")
    time.sleep(8) 

    # 2. Test jmove_async to bin_hard_above
    planner.get_logger().info("Testing jmove_async to bin_hard_above...")
    f_hard_above = planner.jmove_async(bin_hard_above, duration=6.0)
    if f_hard_above: # Corrected variable
        rclpy.spin_until_future_complete(planner, f_hard_above)
        planner.get_logger().info("Controller accepted hard_above command.")
    time.sleep(7)

    # 3. Final Move (Sync) 
    planner.get_logger().info("Plan and move to bin_hard (Sync)...")
    planner.jmove_plan_sync(bin_hard)

    planner.get_logger().info("Direct go to bin_soft (Sync)...")
    planner.jmove_sync(bin_soft)

    # 4. Corrected second async test
    planner.get_logger().info("Testing jmove_async back to bin_hard_above...")
    f_hard_again = planner.jmove_async(bin_hard_above, duration=6.0)
    if f_hard_again: # Corrected variable
        rclpy.spin_until_future_complete(planner, f_hard_again)
        planner.get_logger().info("Controller accepted second hard_above command.")
    time.sleep(7)

    # 5. Corrected third async test
    planner.get_logger().info("Testing jmove_async to bin_soft..")
    f_soft = planner.jmove_async(bin_soft, duration=6.0)
    if f_soft: # Corrected variable
        rclpy.spin_until_future_complete(planner, f_soft)
        planner.get_logger().info("Controller accepted soft command.")
    time.sleep(7)

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
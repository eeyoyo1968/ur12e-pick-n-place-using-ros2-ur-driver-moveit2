import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
import time
import numpy as np
import math

class UR12ePlanner(Node):
    def __init__(self):
        super().__init__('ur12e_planner')
        
        # This client talks to MoveGroup (MoveIt 2) instead of the raw controller
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.get_logger().info("UR12e Planner Node Initialized")

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

    def jmove_plan(self, joint_vector):
        """
        Asks MoveIt 2 to plan and execute a collision-free path to a joint target.
        """
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # Define Joint Constraints for the target
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        constraints = Constraints()
        for name, position in zip(joint_names, joint_vector):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(position)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        # Set Planning Parameters
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        
        self.get_logger().info(f'Planning path to: {joint_vector}...')
        return self._move_group_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    planner = UR12ePlanner()
    
    # Wait for joint states
    start_time = time.time()
    while planner.current_joints is None and (time.time() - start_time) < 5.0:
        rclpy.spin_once(planner, timeout_sec=0.1)
    
    # Target joint_vector (example: your saved Home point)
    home = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
    # --- Math Constants ---
    PI = math.pi
    D2R = PI / 180.0
    #bin_hard=np.array([330.0, -65.72, -140.5, -63.9, 90.0, 60.0])*D2R
    bin_soft=np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0])*D2R  

    bin_hard=np.array([350.0, -100.0, -128.0, -41.5, 90.0, 81.0])*D2R   
    bin_hard_above=np.array([350.0, -90.0, -121.65, -56.14, 90.0, 80.5])*D2R 

    future = planner.jmove_plan(bin_hard)
    future = planner.jmove_plan(home)

    if future:
        rclpy.spin_until_future_complete(planner, future)
        handle = future.result()
        if handle.accepted:
            self_get_result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(planner, self_get_result_future)
            planner.get_logger().info("MoveIt 2 Execution Complete.")
        else:
            planner.get_logger().error("Goal Rejected (Likely Collision or Out of Reach)")
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
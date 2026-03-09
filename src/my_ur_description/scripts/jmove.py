import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
import math

class UR12eCustomMover(Node):
    def __init__(self):
        super().__init__('ur12e_custom_mover')
        # Using the action server found in your working script
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.get_logger().info("UR12e Mover Node Initialized")

    def joint_cb(self, msg):
        # Map names to positions so order doesn't matter
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            # Extract in the exact order the UR driver expects
            self.current_joints = [
                name_to_pos['shoulder_pan_joint'],
                name_to_pos['shoulder_lift_joint'],
                name_to_pos['elbow_joint'],
                name_to_pos['wrist_1_joint'],
                name_to_pos['wrist_2_joint'],
                name_to_pos['wrist_3_joint']
            ]
        except KeyError as e:
            self.get_logger().error(f'Missing joint in /joint_states: {e}')

    def jmove(self, joint_vector, duration=5.0):
        """
        Moves the robot to a specific joint configuration.
        joint_vector: [j1, j2, j3, j4, j5, j6] in radians.
        duration: time in seconds to complete the move.
        """
        if self.current_joints is None:
            self.get_logger().warn('Waiting for joint states... Cannot move.')
            return None

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server not available!')
            return None

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # POINT 1: Start at current position at Time 0
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_joints
        start_point.time_from_start.sec = 0
        
        # POINT 2: Target position at specified duration
        target_point = JointTrajectoryPoint()
        target_point.positions = [float(j) for j in joint_vector]
        target_point.time_from_start.sec = int(duration)
        target_point.time_from_start.nanosec = int((duration % 1) * 1e9)

        goal_msg.trajectory.points = [start_point, target_point]
        
        self.get_logger().info(f'Sending jmove to: {joint_vector}')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    mover = UR12eCustomMover()
    
    # Wait for joint states to be received
    import time
    start_time = time.time()
    while mover.current_joints is None and (time.time() - start_time) < 5.0:
        rclpy.spin_once(mover, timeout_sec=0.1)
    
    # --- TEST: Move to Home Position ---
    # Using your defined home position
    home_position = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
    # --- Math Constants ---
    PI = math.pi
    D2R = PI / 180.0
    #bin_hard=np.array([330.0, -65.72, -140.5, -63.9, 90.0, 60.0])*D2R
    bin_soft=np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0])*D2R  

    bin_hard=np.array([350.0, -100.0, -128.0, -41.5, 90.0, 81.0])*D2R   
    bin_hard_above=np.array([350.0, -90.0, -121.65, -56.14, 90.0, 80.5])*D2R 

    #future = mover.jmove(home_position, duration=8.0)

    future = mover.jmove(bin_hard_above, duration=8.0)
    
    if future:
        rclpy.spin_until_future_complete(mover, future)
        self_get_result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(mover, self_get_result_future)
        mover.get_logger().info("Move Complete.")
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
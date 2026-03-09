import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class UR12eCustomHomeMover(Node):
    def __init__(self):
        super().__init__('ur12e_custom_mover')
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

    def joint_cb(self, msg):
        # 1. Map names to positions so order doesn't matter
        name_to_pos = dict(zip(msg.name, msg.position))
        
        try:
            # 2. Extract them in the EXACT order the UR driver expects
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

    def send_slow_home_goal(self):
        if self.current_joints is None:
            self.get_logger().warn('Waiting for joint states...')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # POINT 1: Current position (Corrected Order) at Time 0
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_joints
        start_point.time_from_start.sec = 0
        
        # POINT 2: Target Home at Time 20
        home_point = JointTrajectoryPoint()
        home_point.positions = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        home_point.time_from_start.sec = 10

        goal_msg.trajectory.points = [start_point, home_point]
        
        self.get_logger().info('Sending mapped trajectory...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    mover = UR12eCustomHomeMover()
    
    # Wait briefly for the joint_states subscriber to receive data
    import time
    start_time = time.time()
    while mover.current_joints is None and (time.time() - start_time) < 5.0:
        rclpy.spin_once(mover, timeout_sec=0.1)
    
    future = mover.send_slow_home_goal()
    
    if future:
        rclpy.spin_until_future_complete(mover, future)
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
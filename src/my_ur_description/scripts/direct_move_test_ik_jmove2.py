import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class ToleranceFixer(Node):
    def __init__(self):
        super().__init__('tolerance_fixer')
        self.joint_pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10
        )

    def sync_and_move(self):
        self.get_logger().info("Clearing tolerance errors...")
        # Send a tiny 'dummy' move to reset the controller state
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        # USE YOUR KNOWN HOME POSITION
        point.positions = [4.7124, -0.5236, -2.618, -1.5707, 1.5707, 0.0]
        point.time_from_start.sec = 1
        msg.points.append(point)
        
        self.joint_pub.publish(msg)
        self.get_logger().info("Controller synced. Now run your Cartesian test.")

def main():
    rclpy.init()
    node = ToleranceFixer()
    node.sync_and_move()
    time.sleep(2.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
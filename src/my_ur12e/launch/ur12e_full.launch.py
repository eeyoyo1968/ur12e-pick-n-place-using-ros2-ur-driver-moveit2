from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Change these to match your setup
    robot_ip = "192.168.2.2" 
    ur_type = "ur12e"

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", PathJoinSubstitution([get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"]),
        " robot_ip:=", robot_ip,
        " ur_type:=", ur_type,
    ])

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": robot_description_content}]),
        Node(package="controller_manager", executable="ros2_control_node", parameters=[{"robot_description": robot_description_content}, 
             PathJoinSubstitution([get_package_share_directory("your_pkg"), "config", "ur_controllers.yaml"])]),
        # Spawners
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["scaled_joint_trajectory_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["force_torque_sensor_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["ur_script_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["io_and_status_controller"]),
    ])
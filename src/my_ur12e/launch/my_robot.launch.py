import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    description_pkg = "my_ur_description" 

    # 1. Setup Configurations
    use_mock_hardware_config = LaunchConfiguration("use_mock_hardware")
    robot_ip_config = LaunchConfiguration("robot_ip")
    kinematics_params_config = LaunchConfiguration("kinematics_params")

    # 2. Define the Robot Description (XACRO)
    # This maps the launch arguments to the xacro:args defined in your .xacro file
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", PathJoinSubstitution([get_package_share_directory(description_pkg), "urdf", "ur_minimal.xacro"]),
        " robot_ip:=", robot_ip_config,
        " use_mock_hardware:=", use_mock_hardware_config,
        " kinematics_params:=", kinematics_params_config,
    ])
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.2.2"),
        DeclareLaunchArgument("kinematics_params", 
                              default_value=os.path.join(get_package_share_directory("ur_description"), "config", "ur12e", "default_kinematics.yaml")),

        # 3. ROBOT STATE PUBLISHER
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),

        # 4. ROS 2 CONTROL NODE
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description, 
                PathJoinSubstitution([get_package_share_directory(description_pkg), "config", "ur_controllers.yaml"])
            ],
            output="both",
        ),

        # 5. SPAWNERS
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["scaled_joint_trajectory_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["force_torque_sensor_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["ur_script_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["io_and_status_controller"]),
    ])
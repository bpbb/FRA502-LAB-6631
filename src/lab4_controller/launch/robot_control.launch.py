import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    """
    Launch file for Lab 4: 3R Robot Control System (3-node architecture)
    """
    
    # Declare arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Launch joint_state_publisher_gui for manual control'
    )
    
    # Get launch configuration
    use_gui = LaunchConfiguration('use_gui')
    
    # Package name
    pkg = get_package_share_directory("lab4_controller")
    
    # RVIZ configuration
    rviz_path = os.path.join(pkg, "rviz", "config_rviz.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_path],
        output="screen",
    )
    
    # Robot description from XACRO
    path_description = os.path.join(pkg, "urdf", "my-robot.xacro")
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{"robot_description": robot_desc_xml}]
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=parameters,
    )
    
    # Robot scheduler node (main coordinator)
    robot_scheduler_node = Node(
        package='lab4_controller',
        executable='scheduler.py',
        name='scheduler_node',
        output='screen'
    )
    
    # Controller node (motion control)
    controller_node = Node(
        package='lab4_controller',
        executable='controller.py',
        name='controller_node',
        output='screen'
    )
    
    # Random pose generator node
    random_node = Node(
        package='lab4_controller',
        executable='random_target.py',
        name='random_node',
        output='screen'
    )
    
    # Teleop keyboard node (optional - start separately)
    teleop_node = Node(
        package='lab4_controller',
        executable='teleop_jog_keyboard.py',
        name='teleop_keyboard',
        output='screen'
    )
    
    # Joint state publisher GUI (optional)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher,
        rviz,
        robot_scheduler_node,
        controller_node,
        random_node,
        teleop_node,
        joint_state_publisher_gui,
    ])
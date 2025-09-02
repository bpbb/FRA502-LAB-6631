import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    turtle1_namespace = 'eater_beam'
    turtle2_namespace = 'killer_beam'
        
    turtlesim_plus = Node(
        package='turtlesim_plus',
        executable='turtlesim_plus_node.py',
        name='turtlesim',
        output='screen',
    )

    turtle1 = Node(
        package='lab3',
        executable='eater.py',
        namespace=turtle1_namespace,
        output='screen',
        parameters=[{'sampling_frequency': 50.0}],
    )

    turtle2 = Node(
        package='lab3',
        executable='killer.py',
        namespace=turtle2_namespace,
        output='screen',
        parameters=[{
            'sampling_frequency': 50.0,
            'kill': turtle1_namespace
            }],
    )

    kill_turtle1 = ExecuteProcess(
        cmd = ['ros2', 'service', 'call', '/remove_turtle', 'turtlesim/srv/Kill', '{name: \'turtle1\'}'],
        output='screen'
    )

    spawn_turtle1 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_turtle', 'turtlesim/srv/Spawn', f'"{{x: 7.0, y: 7.0, theta: 0.0, name: {turtle1_namespace}}}"'],
        output='screen',
        shell=True
    )

    spawn_turtle2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_turtle', 'turtlesim/srv/Spawn', f'"{{x: 2.0, y: 2.0, theta: 0.0, name: {turtle2_namespace}}}"'],
        output='screen',
        shell=True
    )

    launch_description = LaunchDescription()
    launch_description.add_action(turtlesim_plus)
    launch_description.add_action(turtle1)
    launch_description.add_action(turtle2)
    launch_description.add_action(kill_turtle1)
    launch_description.add_action(spawn_turtle1)
    launch_description.add_action(spawn_turtle2)
    
    return launch_description
 

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == "__main__":
    main()




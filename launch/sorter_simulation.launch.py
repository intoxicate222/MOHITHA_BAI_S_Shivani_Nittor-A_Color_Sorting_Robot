import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_sorter_sim')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'simple_arm.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'sorter_sim.rviz')

    
    if not os.path.exists(urdf_file_path):
        raise EnvironmentError(f"URDF file not found: {urdf_file_path}")

    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

   
    if not os.path.exists(rviz_config_path):
        print(f"Warning: RViz config file not found: {rviz_config_path}. RViz will start with default config.")
        rviz_args = []
    else:
        rviz_args = ['-d', rviz_config_path]


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
    )

    # ENABLED for manual control
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args 
    )

    sorter_logic_node = Node(
        package='simple_sorter_sim',
        executable='sorter_logic.py', 
        name='sorter_logic_node',
        output='screen'
    )

    
    nodes_to_launch = [
        robot_state_publisher_node,
        #joint_state_publisher_gui_node, # Make sure this is IN the list
        rviz_node,
        sorter_logic_node,
    ]

    return LaunchDescription(nodes_to_launch)
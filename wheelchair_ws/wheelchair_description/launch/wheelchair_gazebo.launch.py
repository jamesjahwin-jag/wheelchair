# GAZEBO LAUNCH FILE OF WHEELCHAIR
# Made by Jahwin James James
# With reference to the youtube video: https://www.youtube.com/watch?v=V9ztoMuSX8w&t=68s -->
# On 17/02/2026 4:28
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Configuration ---
    robotXacroname = 'wheelchair'
    namePackage = 'wheelchair_description'
    modelFileRelativePath = 'urdf/wheelchairtest.xacro'

    # --- Path Resolution ---
    pkg_share = get_package_share_directory(namePackage)
    pathModelFile = os.path.join(pkg_share, modelFileRelativePath)

    # --- Process XACRO ---
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # --- Gazebo Launch ---
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, 
        launch_arguments={
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )

    # --- Nodes ---
    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription, 
            'use_sim_time': True
        }]
    )

    # 2. Spawn Entity
    spawnModelNode = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroname,'-z', '2'],
        output='screen'
    )

    # --- Return Launch Description ---
    return LaunchDescription([
        gazeboLaunch,
        node_robot_state_publisher,
        spawnModelNode
    ])
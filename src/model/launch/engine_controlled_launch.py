from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os, xacro

def generate_launch_description():
    ld = LaunchDescription()
    
    share_dir = get_package_share_directory('model')

    # Get robot description and rviz config
    rviz_config =  os.path.join(share_dir, "rviz/engine_controlled.rviz")

    xacro_file = os.path.join(share_dir, 'urdf', 'fusion_parts_model.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description_content = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
            }
        ],
    )

    ld.add_action(robot_state_publisher_node)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', str(rviz_config)]
    )

    ld.add_action(rviz)

    cup_publisher = Node(
        package="model",
        executable="cup_publisher"
    )

    ld.add_action(cup_publisher)
    
    
    engine = Node(
        package="model",
        executable="engine"
    )

    ld.add_action(engine)
    
    return ld
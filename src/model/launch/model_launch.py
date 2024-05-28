from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os

def generate_launch_description():
    ld = LaunchDescription()

    model = FindPackageShare(package="model").find("model")

    # Get the paths for URDF model and RViz configuration
    my_model_path = os.path.join(model, "urdf/model.urdf")
    
    robot_description_content = ParameterValue(
        Command(["xacro ", my_model_path]), value_type=str
    )

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
        output="screen"
    )

    ld.add_action(rviz)
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )
    
    # ld.add_action(joint_state_publisher_gui_node)

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

    nozzle_position = Node(
        package="model",
        executable="nozzle_position"
    )

    ld.add_action(nozzle_position)

    return ld
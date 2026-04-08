import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic", default="true")
    robot_description_path = get_package_share_directory("abb_irb1200_support") # originally abb_irb1200_support

    robot_description_config = xacro.process_file(
        os.path.join(
            robot_description_path,
            "urdf",                 # originally urdf
            "irb1200_5_90.xacro",   # originally irb1200_5_90.xacro
        )
    )

    robot_description = {"robot_description": robot_description_config.toxml()}
    
    # robot_description_semantic = {"robot_description_semantic": xacro.process_file(
    #     os.path.join(
    #         robot_description_path,
    #         "config",                # originally config
    #         "abb_irb1200_5_90.srdf.xacro", # originally abb_irb1200_5_90.srdf.xacro
    #     )
    # ).toxml()}

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="abb_irb1200_5_90", # originally abb_irb1200_5_90
            package_name="abb_irb1200_5_90_moveit_config" # originally abb_irb1200_5_90_moveit_config
        )
        .robot_description_semantic(file_path=os.path.join(get_package_share_directory("abb_irb1200_5_90_moveit_config"), #gemini recommended
            "config", 
            "abb_irb1200_5_90.srdf.xacro")) 
        .robot_description_kinematics(file_path="config/kinematics.yaml") #gemini recommended
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(file_path="config/motion_planning.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl"
        ).to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    moveit_node = Node(
        name = "moveit",
        package = "cutting_robot",
        executable = "moveit_node", # change to match the left side in set up file
        parameters = [moveit_config.to_dict()]
    )

    # rviz_node = Node(
    #     package = "rviz2",
    #     executable = "rviz2",
    #     name = "rviz2",
    #     output="log",
    #     parameters = [
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #     ],
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(robot_description_path, "rviz", "urdf_description.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description], # originally [robot_description]
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("abb_bringup"), # Replace with abb_bring up package
        "config",
        "abb_controllers.yaml", # abb_controllers.yaml
    )

    ros2_controllers_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )

    controller_spawners = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
        ]
        + ['joint_trajectory_controller', 'joint_state_broadcaster'],
        output="both",
    )

    joint_state_sliders = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    return LaunchDescription([
        move_group_node,
        moveit_node,
        robot_state_publisher,
        ros2_controllers_node,
        rviz_node,
        controller_spawners,
        #joint_state_sliders,
    ] 
    )
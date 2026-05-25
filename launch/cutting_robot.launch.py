import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction #ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

# New Launch setup (mutiple robots)
def launch_setup(context, *args, **kwargs):

    robot = LaunchConfiguration("robot").perform(context)

    robot_description_config = None
    moveit_config = None
    rviz_config = None

    # Robot Specific configuration
    if robot == "ur5e":
        robot_description_path = get_package_share_directory("ur_description")
        ur_moveit_config_path = get_package_share_directory("ur_moveit_config")
        
        xacro_file = os.path.join(robot_description_path, "urdf", "ur.urdf.xacro")
        srdf_file = os.path.join(ur_moveit_config_path, "srdf", "ur.srdf.xacro")
        
        robot_description_config = xacro.process_file(
            xacro_file,
            mappings={
                        "ur_type": "ur5e", 
                        "name": "ur", 
                        "use_mock_hardware": "true",
                        "tf_prefix": ""
                    }
        )
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="ur",
                package_name="ur_moveit_config"
            )
            .robot_description(
                file_path=xacro_file,
                mappings={"ur_type": "ur5e", 
                          "name": "ur", 
                          "use_mock_hardware": "true"
                        }
            )
            .robot_description_semantic(
                file_path=srdf_file,
                mappings={"name": "ur"}
            )
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .moveit_cpp(file_path="config/motion_planning.yaml")
            .planning_pipelines(
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
                default_planning_pipeline="ompl"
            )
            .to_moveit_configs()
        )
        rviz_config = os.path.join(ur_moveit_config_path, "config", "moveit.rviz")

    elif robot == "ur10e":
        robot_description_path = get_package_share_directory("ur_description")
        ur_moveit_config_path = get_package_share_directory("ur_moveit_config")
        xacro_file = os.path.join(robot_description_path, "urdf", "ur.urdf.xacro")
        srdf_file = os.path.join(ur_moveit_config_path, "srdf", "ur.srdf.xacro")
        robot_description_config = xacro.process_file(
            xacro_file,
            mappings={"ur_type": "ur10e", "name": "ur", "use_mock_hardware": "true", "tf_prefix": ""}
        )
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="ur",
                package_name="ur_moveit_config"
            )
            .robot_description(
                file_path=xacro_file,
                mappings={"ur_type": "ur10e", "name": "ur", "use_mock_hardware": "true",}
            )
            .robot_description_semantic(
                file_path=srdf_file,
                mappings={"name": "ur"}
            )
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .moveit_cpp(file_path="config/motion_planning.yaml")
            .planning_pipelines(
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
                default_planning_pipeline="ompl"
            )
            .to_moveit_configs()
        )
        rviz_config = os.path.join(ur_moveit_config_path, "config", "moveit.rviz")

    else:
        robot_description_path = get_package_share_directory("abb_irb1200_support")
        robot_description_config = xacro.process_file(
            os.path.join(robot_description_path, "urdf", "irb1200_5_90.xacro")
        )
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="abb_irb1200_5_90",
                package_name="abb_irb1200_5_90_moveit_config"
            )
            .robot_description_semantic(file_path=os.path.join(
                get_package_share_directory("abb_irb1200_5_90_moveit_config"),
                "config",
                "abb_irb1200_5_90.srdf.xacro"))
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .moveit_cpp(file_path="config/motion_planning.yaml")
            .planning_pipelines(
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
                default_planning_pipeline="ompl"
            )
            .to_moveit_configs()
        )
        rviz_config = os.path.join(robot_description_path, "rviz", "urdf_description.rviz")

    robot_description_xml = robot_description_config.toxml()
    robot_description = {"robot_description": robot_description_xml}
    robot_description_param = {"robot_description": robot_description_xml}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": "true"},
        ],
    )

    unified_moveit_params = {
        **moveit_config.to_dict(),
        **robot_description_param,
        "use_sim_time": True
    }

    moveit_node = Node(
        name = "moveit_cpp",
        package = "cutting_robot",
        executable = "moveit_node", # change to match the left side in set up file
        parameters = [
            moveit_config.to_dict(),
            robot_description_param,
            {"use_sim_time": True}
        ],
        arguments=[f"robot_type:={robot}"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            rviz_config
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

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("abb_bringup"), # Replace with abb_bring up package
    #     "config",
    #     "abb_controllers.yaml", # abb_controllers.yaml
    # )

    if "ur" in robot:
        ros2_controllers_path = os.path.join(
            get_package_share_directory("ur_robot_driver"),
            "config",
            "ur_controllers.yaml"
        )
    else:
        ros2_controllers_path = os.path.join(
            get_package_share_directory("abb_bringup"),
            "config",
            "abb_controllers.yaml"
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

    return [
        move_group_node,
        moveit_node,
        robot_state_publisher,
        ros2_controllers_node,
        rviz_node,
        controller_spawners,
        #joint_state_sliders,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="abb_irb1200",
            description="Robot to use: ur5e, ur10e, or abb_irb1200"
        ),
        DeclareLaunchArgument(
            "publish_robot_description_sematic",
            default_value="true"
        ),
        OpaqueFunction(function=launch_setup)
    ])
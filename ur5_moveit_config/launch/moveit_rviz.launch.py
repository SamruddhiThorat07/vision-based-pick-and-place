import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="ur5_moveit_config")
        .robot_description(file_path="config/ur5.urdf.xacro")
        .robot_description_semantic(file_path="config/ur5.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(default_planning_pipeline="ompl",pipelines=["ompl"])
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}, 
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            # {"trajectory_execution.allowed_start_tolerance": 0.2},
        ],
    )

    rviz_config_file = os.path.join(
        moveit_config.package_path, "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )
    
    moveit_interface_node = TimerAction(
        period = 8.0,
        actions= [
            Node(
        package="ur5_arm",
        executable="move_arm_obj.py",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )]
    )
    
    pymoveit_interface_node = TimerAction(
        period = 5.0,
        actions= [
            Node(
        package="ur5_arm",
        executable="moveit_grasp.py",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
        # moveit_interface_node,
        pymoveit_interface_node
    ])
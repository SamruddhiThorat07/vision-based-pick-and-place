from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    random_arg = DeclareLaunchArgument("random", default_value="false", description="place the objects at random positions")
    seed_arg = DeclareLaunchArgument("seed", default_value="-1", description="seed for the random layout (-1 = new random seed)")

    # gazebo world, robot, controllers and point cloud node
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch", "spawn_ur5.launch.py"])
        )
    )

    # moveit, rviz and the pick and place node
    # waits for the controllers in spawn_ur5 to be active (last one starts at 7s)
    moveit_launch = TimerAction(
        period = 10.0,
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("ur5_moveit_config"), "launch", "moveit_rviz.launch.py"])
                )
            )
        ]
    )

    # moves the objects after gazebo is up but before the pick and place node starts (at 15s)
    randomize_node = TimerAction(
        period = 8.0,
        actions = [
            Node(
                package="ur5_arm",
                executable="randomize_objects.py",
                output="screen",
                parameters=[{"seed": LaunchConfiguration("seed")}],
                condition=IfCondition(LaunchConfiguration("random"))
            )
        ]
    )

    return LaunchDescription([
        random_arg,
        seed_arg,
        sim_launch,
        randomize_node,
        moveit_launch
    ])

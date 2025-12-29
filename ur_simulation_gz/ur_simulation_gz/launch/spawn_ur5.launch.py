#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    AppendEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="Type of UR robot",
        )
    )

    # Get configurations
    ur_type = LaunchConfiguration("ur_type")

    # Package paths
    ur_description_package = FindPackageShare("ur_description")
    ur_simulation_gz_package = FindPackageShare("ur_simulation_gz")
    
    pkg_share = get_package_share_directory("ur_description")
    parent_dir = os.path.dirname(pkg_share)
    
    resource_env = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=parent_dir,
    )
    
    world_launch_path = PathJoinSubstitution([ur_description_package, "launch", "world.launch.py"])
    
    launch_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_path)
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([ur_description_package, "urdf", "ur.urdf.xacro"]),
        " ",
        "name:=ur5",
        " ",
        "ur_type:=", ur_type,
        " ",
        # "prefix:=ur5_", 
        " ",
        "sim_ignition:=true",
        " ",
        "use_fake_hardware:=true",
        " ",
        "fake_sensor_commands:=false",
        " ",
        "initial_positions_file:=", PathJoinSubstitution([ur_description_package, "config", "initial_positions.yaml"]),
        " ",
        "simulation_controllers:=", PathJoinSubstitution([ur_simulation_gz_package, "config", "ur_controllers.yaml"]),
    ])

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                # namespace="ur5",  # Add namespace
                output="both",
                parameters=[
                    robot_description,
                    {
                        "use_sim_time": True,
                        # "frame_prefix": "ur5_"  # Add frame prefix
                    }
                ],
            )
        ]
    )

    bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ur5_gz_bridge",
                # namespace="ur5",  
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                    "/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                    "/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                    "/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
                    "/world/table_world/model/ur5/link/robotiq_85_left_finger_tip_link/sensor/left_contact_sensor/contact@sensor_msgs/msg/ContactsState[gazebosim.msgs.Contacts",
                    "/world/table_world/model/ur5/link/robotiq_85_right_finger_tip_link/sensor/right_contact_sensor/contact@sensor_msgs/msg/ContactsState[gazebosim.msgs.Contacts",   ],
                remappings=[
                    ("/camera/image", "/camera/image_raw"),
                    ("/camera/depth_image", "/camera/depth/image_raw"),
                    ("/camera/camera_info", "/camera/camera_info"),
                    ("/camera/points", "/camera/depth/points"),
                ],
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    spawn_robot = TimerAction(
        period=3.5,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_ur5",
                output="screen",
                arguments=[
                    "-name", "ur5",
                    "-topic", "robot_description",  
                    "-x", "0.0",
                    "-y", "-0.60",
                    "-z", "1.05",
                    "-R", "0.0",
                    "-P", "0.0",
                    "-Y", "0.0",
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster", 
                    "--controller-manager", "/controller_manager" 
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ]
    )

    forward_velocity_controller_spawner = TimerAction(
        period=5.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "forward_velocity_controller", 
                    "--controller-manager", "/controller_manager"  
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ]
    )
    
    joint_trajectory_controller_spawner = TimerAction(
        period = 6.0,
        actions = [
            Node(
                package = "controller_manager",
                executable="spawner",
                arguments=[
                    "joint_trajectory_controller",
                    "--controller-manager", "/controller_manager"
                ],
                parameters=[{"use_sim_time": True}],
                output = "screen",
            )
        ]
    )
    
    robotiq_gripper_controller_spawner = TimerAction(
        period = 6.5,
        actions = [
            Node(
                package = "controller_manager",
                executable="spawner",
                arguments=[
                    "robotiq_gripper_controller",
                    "--controller-manager", "/controller_manager"
                ],
                parameters=[{"use_sim_time": True}],
                output = "screen",
            )
        ]
    )
    
    rviz_config = os.path.join(get_package_share_directory("ur_simulation_gz"),"config","camera.rviz")
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output = "screen",
        arguments=["-d",rviz_config]
    )
    
    point_cloud_node =TimerAction(
        period = 7.0,
        actions = [
            Node(
        package="ur5_arm",
        executable="point_cloud.py",
        output = "screen",
    )])
    
    return LaunchDescription(
        declared_arguments + [
            resource_env,
            launch_world,                   
            robot_state_publisher_node,     
            bridge,                         
            spawn_robot,            
            joint_state_broadcaster_spawner,
            # forward_velocity_controller_spawner,
            joint_trajectory_controller_spawner,
            robotiq_gripper_controller_spawner,
            # rviz2,
            point_cloud_node
        ]
    )
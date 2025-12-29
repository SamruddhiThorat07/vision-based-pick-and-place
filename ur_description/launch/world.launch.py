#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable,AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('ur_description')

    gz_resource_path = os.path.dirname(pkg_share)
    
    plugin_path = os.path.join(pkg_share, 'plugins')
    
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )
    
    plugin_env = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_path
    )
    
    set_env_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                                  os.path.join(get_package_share_directory('ur_description'), 'world'))
    
    

    sdf_file = os.path.join(pkg_share, 'world', 'table_world.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 "{sdf_file}"'}.items()
    )

    return LaunchDescription([
        set_env_resources,
        resource_env,
        plugin_env,
        gz_sim,
    ])
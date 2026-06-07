# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Simulation World'),
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('gui_config', default_value='',
                          description='Optional full path to a Gazebo GUI config.'),
]


def generate_launch_description():

    # Directories
    pkg_turtlebot4_gz_bringup = get_package_share_directory(
        'turtlebot4_gz_bringup')
    pkg_turtlebot4_gz_gui_plugins = get_package_share_directory(
        'turtlebot4_gz_gui_plugins')
    pkg_particlefilter_gz_gui_plugins = get_package_share_directory(
        'particlefilter_gz_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_irobot_create_gz_bringup = get_package_share_directory(
        'irobot_create_gz_bringup')
    pkg_irobot_create_gz_plugins = get_package_share_directory(
        'irobot_create_gz_plugins')
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_turtlebot4_gz_bringup, 'worlds'),
            os.path.join(pkg_turtlebot4_gz_bringup, 'models'),
            os.path.join(pkg_irobot_create_gz_bringup, 'worlds'),
            str(Path(pkg_turtlebot4_description).parent.resolve()),
            str(Path(pkg_irobot_create_description).parent.resolve()),
            os.path.expanduser('~/Downloads/small_house'),
            os.path.expanduser('~/Downloads/small_house/models'),
            os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/openrobotics/models'),
            os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/movai/models'),
            os.path.expanduser('~/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models'),
            os.path.expanduser('~/.gz/fuel/fuel.ignitionrobotics.org/makerspet/models')
        ])
    )

    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_particlefilter_gz_gui_plugins, 'lib'),
            os.path.join(pkg_turtlebot4_gz_gui_plugins, 'lib'),
            os.path.join(pkg_irobot_create_gz_plugins, 'lib')
        ])
    )

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    default_gui_config = PathJoinSubstitution([
        pkg_turtlebot4_gz_bringup,
        'gui',
        LaunchConfiguration('model'),
        'gui.config'
    ])

    # Gazebo harmonic
    default_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('gui_config'), "' == ''"])),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -r',
                ' -v 4',
                ' --render-engine ogre2 ',
                ' --gui-config ',
                default_gui_config
            ])
        ]
    )

    custom_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('gui_config'), "' != ''"])),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -r',
                ' -v 4',
                ' --render-engine ogre2 ',
                ' --gui-config ',
                LaunchConfiguration('gui_config')
            ])
        ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gz_gui_plugin_path)
    ld.add_action(default_gazebo)
    ld.add_action(custom_gazebo)
    ld.add_action(clock_bridge)
    return ld

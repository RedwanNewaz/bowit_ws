from launch import LaunchDescription
from launch_ros.actions import Node


import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory


def getRobot(current_pkg_dir, name):
    param_path = os.path.join(current_pkg_dir, 'config', '%s_param.yaml' % name)
    
    with open(param_path) as file:
        param = yaml.safe_load(file)
    
    param['coord'] = os.path.join(current_pkg_dir, 'config', param['coord'])
    param['data'] = os.path.join(current_pkg_dir, 'config', param['data'])
    robot = Node(package='bowit',
        namespace=name,
        executable='bowit_node',
        name='bowit_robot',
        parameters=[param],
        output='screen'
    )

    return robot  


def generate_launch_description():
    ld = LaunchDescription()
    current_pkg_dir = get_package_share_directory('bowit')
    current_pkg_dir = str(current_pkg_dir)

    max_num_robots = 4

    robots = [ getRobot(current_pkg_dir, "robot%d" % (i + 1))  for i in range(max_num_robots)] 

    ld = LaunchDescription(robots)

    viz = Node(package='bowit',
        executable='bowit_viz',
        name='bowit_viz'
        )
    ld.add_action(viz)
    
    param_path = os.path.join(current_pkg_dir, 'config', 'bowit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='bowit_rviz2',
        arguments=['-d', param_path]
    )
    ld.add_action(rviz)

    return ld


   
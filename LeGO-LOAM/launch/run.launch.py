import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import expanduser

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import yaml

def generate_launch_description():

  # Configure environment
  stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
  stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

  # Simulated time
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  # Nodes Configurations
  config_file = os.path.join("/home/harry/dev_ws/src/LeGO-LOAM-SR/LeGO-LOAM/config", 'loam_config.yaml')
  #config_file = os.path.join(get_package_share_directory('lego_loam_sr'), 'config', 'loam_config.yaml')
  rviz_config = os.path.join(get_package_share_directory('lego_loam_sr'), 'rviz', 'test.rviz')

  # Tf transformations
  # initialization of camera_init by using UWB
  # 1.91, 2.43, 0.84,
  transform_map = Node(
    package='tf2_ros',
    node_executable='static_transform_publisher',
    node_name='camera_init_to_map',
    arguments=['1.91', '2.43', '0.84', '1.570795', '0', '1.570795', 'map', 'camera_init'],
  )

  transform_camera = Node(
    package='tf2_ros',
    node_executable='static_transform_publisher',
    node_name='base_link_to_camera',
    arguments=['0', '0', '0', '-1.570795', '-1.570795', '0', 'camera', 'base_link'],
  )

  transform_vel = Node(
    package='tf2_ros',
    node_executable='static_transform_publisher',
    node_name='vel_to_camera',
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
  )

  # LeGO-LOAM
  lego_loam_node = Node(
    package='lego_loam_sr',
    node_executable='lego_loam_sr',
    output='screen',
    parameters=[config_file],
    remappings=[('/lidar_points', '/velodyne_points')],
  )

  # Rviz
  rviz_node = Node(
    package='rviz2',
    node_executable='rviz2',
    node_name='rviz2',
    arguments=['-d', rviz_config],
    output='screen'
  )

  driver_share_dir = get_package_share_directory('velodyne_driver')
  driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
  with open(driver_params_file, 'r') as f:
      driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

  convert_share_dir = get_package_share_directory('velodyne_pointcloud')
  convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_convert_node-params.yaml')
  with open(convert_params_file, 'r') as f:
      convert_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
  convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')

  laserscan_share_dir = get_package_share_directory('velodyne_laserscan')
  laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
  with open(laserscan_params_file, 'r') as f:
      laserscan_params = yaml.safe_load(f)['velodyne_laserscan_node']['ros__parameters']

  container = ComposableNodeContainer(
          node_name='velodyne_container',
          node_namespace='',
          package='rclcpp_components',
          node_executable='component_container',
          composable_node_descriptions=[
              ComposableNode(
                  package='velodyne_driver',
                  node_plugin='velodyne_driver::VelodyneDriver',
                  node_name='velodyne_driver_node',
                  parameters=[driver_params]),
              ComposableNode(
                  package='velodyne_pointcloud',
                  node_plugin='velodyne_pointcloud::Convert',
                  node_name='velodyne_convert_node',
                  parameters=[convert_params]),
              ComposableNode(
                  package='velodyne_laserscan',
                  node_plugin='velodyne_laserscan::VelodyneLaserScan',
                  node_name='velodyne_laserscan_node',
                  parameters=[laserscan_params]),
          ],
          output='both',
  )


  ld = LaunchDescription()
  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(stdout_colorized_envvar)
  # Add nodes
  ld.add_action(lego_loam_node)
  ld.add_action(transform_map)
  ld.add_action(transform_camera)
  ld.add_action(transform_vel)

  ld.add_action(container)
  #ld.add_action(rviz_node)

  return ld

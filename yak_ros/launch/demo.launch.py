import os
import launch
import launch_ros.actions
from launch_ros.substitutions.find_package import get_package_share_directory

def generate_launch_description():
    mesh_path = os.path.join(get_package_share_directory('yak_ros'), 'demo', 'bun_on_table.ply')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            node_name='image_sim_node',
            package='yak_ros',
            node_executable='yak_ros_image_simulator',
            output='screen',
            parameters=[{'mesh': mesh_path,
            'base_frame': 'world',
            'orbit_speed': 1.0,
            'framerate': 30.0}]),
        launch_ros.actions.Node(
            node_name='tsdf_node',
            package='yak_ros',
            node_executable='yak_ros_node',
            output='screen',
            remappings=[('input_depth_image', '/image')],
            parameters=[{'tsdf_frame': 'tsdf_origin',
            'cx': 320.0,
            'cy': 240.0,
            'fx': 550.0,
            'fy': 550.0,
            'cols': 640,
            'rows': 480,
            'volume_resolution': 0.001,
            'volume_x': 640,
            'volume_y': 640,
            'volume_z': 192}]),
          launch_ros.actions.Node(
              node_name='world_to_tsdf_origin_pub',
              package='tf2_ros',
              node_executable='static_transform_publisher',
              output='log',
              arguments=['-0.3', '-0.3', '-0.01', '0', '0', '0', '1', 'world', 'tsdf_origin']),
])

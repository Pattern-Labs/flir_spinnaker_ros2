from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

camera_params = {
    'debug': False,
    'compute_brightness': True,
    'dump_node_map': False,
    # set parameters defined in grasshopper.cfg    
    'gain_auto': 'Off',
    'exposure_auto': 'Off',
    'always_publish': True,
    'frame_rate_auto': 'Off',
    'frame_rate': 20.0,
    'frame_rate_enable': True,
    'trigger_mode': 'Off',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    }

def generate_launch_description():
    """Generate launch description with multiple components."""
    flir_dir = get_package_share_directory('flir_spinnaker_ros2')
    config_dir = flir_dir + '/config/'
    name_arg = LaunchArg('camera_name', default_value='blackfly_s',
                         description='camera name')
    serial_arg = LaunchArg('serial', default_value="'20435008'",
                         description='serial number')
    print([LaunchConfig('serial'),'_'])
    container = ComposableNodeContainer(
            name='blackfly_s_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                        package='flir_spinnaker_ros2',
                        plugin='flir_spinnaker_ros2::CameraDriver',
                        name=[LaunchConfig('camera_name')],
                        parameters=[camera_params,
                                    {'parameter_file': config_dir + 'blackfly_s.cfg',
                                    'serial_number': [LaunchConfig('serial')],
                                    }],
                        remappings=[('~/control', '/blackfly_s/control'), ]),
                ComposableNode(
                        package='flir_spinnaker_ros2',
                        plugin='flir_spinnaker_ros2::CameraAutoExposure',
                        name=['blackfly_s_autoexposure'])
            ],
            output='screen',
    )

    return LaunchDescription([name_arg, serial_arg, container])

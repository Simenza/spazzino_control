import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    package_name = "spazzino_control"


    # Set paths to Xacro model and configuration files
    robot_model_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'robot.xacro'
    )

    

    '''slam_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml'
    )'''

    '''twist_mux_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )

    nav2_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )'''


    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(robot_model_path).toxml()



    # Create a node to publish the robot's state based on its URDF description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False}
        ],
        output='screen'
    )


    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'spazz.ino/body_link/laser_frame'],
        output='screen'
    )

    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join('sllidar_ros2'),
    #         'launch',
    #         'sllidar_c1_launch.py'
    #     )
    # )

    arduino_bridge_node = Node(
            package=package_name,
            executable="arduino_bridge.py",
            name="arduino_bridge",
            output="screen"
    )

    # esp_odom_node = Node(
    #     package='spazzino_control',
    #     executable='esp_odom_node.py',
    #     name='esp_odom_node',
    #     output='screen'
    # )

    # esp_cmdvel_node = Node(
    #     package='spazzino_control',
    #     executable='esp_cmdvel_node.py',
    #     name='esp_cmdvel_node',
    #     output='screen'
    # )


    """slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': 'false'
        }.items()
    )"""


    '''twist_mux_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'twist_mux', 'twist_mux',
            '--ros-args',
            '--params-file', twist_mux_params_path,
            '-r', 'cmd_vel_out:=diff_cont/cmd_vel_unstamped'
        ],
        output='screen'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'false'
        }.items()
    )'''





    return LaunchDescription([
        robot_state_publisher_node,
        #lidar_launch,
        arduino_bridge_node,
        static_tf_pub_node
        #esp_cmdvel_node,
        #esp_odom_node
        #slam_toolbox_launch,
        #twist_mux_process,
        #nav2_launch,
    ])

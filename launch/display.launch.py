import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    urdf_file_name = 'HoverRobot2.urdf'
    urdf = os.path.join(
        get_package_share_directory('hoverrobot_description'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    level_log = 'warn'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf, '--ros-args', '--log-level', level_log],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    staticTransformCollision_FL = Node(             # Transformacion estatica para sensor de colision delantero izquierdo
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.085', '0', '0', '0', '0', 'base_link', 'range_front_left','--ros-args', '--log-level', level_log],
        
    )

    staticTransformCollision_FR = Node(             # Transformacion estatica para sensor de colision delantero derecho
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '-0.085', '0', '0', '0', '0', 'base_link', 'range_front_right','--ros-args', '--log-level', level_log],
    )

    staticTransformCollision_RL = Node(             # Transformacion estatica para sensor de colision trasero izquierdo
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.085', '0', '3.14', '0', '0', 'base_link', 'range_rear_left','--ros-args', '--log-level', level_log],
    )

    staticTransformCollision_RR = Node(             # Transformacion estatica para sensor de colision trasero derecho
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '-0.085', '0', '3.14', '0', '0', 'base_link', 'range_rear_right','--ros-args', '--log-level', level_log],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        robot_state_publisher_node,
        staticTransformCollision_FL,
        staticTransformCollision_FR,
        staticTransformCollision_RL,
        staticTransformCollision_RR,
        # rviz_node,
    ])
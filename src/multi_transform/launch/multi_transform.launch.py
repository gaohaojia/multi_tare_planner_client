from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def get_id_map_trans_publisher(context: LaunchContext, offsetList, robot_id):
    robot_id_str = 'robot_' + context.perform_substitution(robot_id)
    offsetList_str = []
    for idx in offsetList:
        offsetList_str.append(context.perform_substitution(idx))
    sensor_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="idMapTransPublisher",
        arguments=[*offsetList_str, 'map', robot_id_str + '/map']
    )
    return [sensor_trans_publisher]

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    multiOffsetPositionX = LaunchConfiguration('multiOffsetPositionX')
    multiOffsetPositionY = LaunchConfiguration('multiOffsetPositionY')
    multiOffsetPositionZ = LaunchConfiguration('multiOffsetPositionZ')
    multiOffsetRotateR = LaunchConfiguration('multiOffsetRotateR')
    multiOffsetRotateP = LaunchConfiguration('multiOffsetRotateP')
    multiOffsetRotateY = LaunchConfiguration('multiOffsetRotateY')
    
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')
    declare_multiOffsetPositionX = DeclareLaunchArgument('multiOffsetPositionX', default_value='0.0', description='')
    declare_multiOffsetPositionY = DeclareLaunchArgument('multiOffsetPositionY', default_value='0.0', description='')
    declare_multiOffsetPositionZ = DeclareLaunchArgument('multiOffsetPositionZ', default_value='0.0', description='')
    declare_multiOffsetRotateR = DeclareLaunchArgument('multiOffsetRotateR', default_value='0.0', description='')
    declare_multiOffsetRotateP = DeclareLaunchArgument('multiOffsetRotateP', default_value='0.0', description='')
    declare_multiOffsetRotateY = DeclareLaunchArgument('multiOffsetRotateY', default_value='0.0', description='')

    multi_transform_node = Node(
        package='multi_transform',
        executable='multi_transform_node',
        name='multi_transform',
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            'multiOffsetPositionX': multiOffsetPositionX,
            'multiOffsetPositionY': multiOffsetPositionY,
            'multiOffsetPositionZ': multiOffsetPositionZ,
            'multiOffsetRotateR': multiOffsetRotateR,
            'multiOffsetRotateP': multiOffsetRotateP,
            'multiOffsetRotateY': multiOffsetRotateY,
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_id)
    ld.add_action(declare_multiOffsetPositionX)
    ld.add_action(declare_multiOffsetPositionY)
    ld.add_action(declare_multiOffsetPositionZ)
    ld.add_action(declare_multiOffsetRotateR)
    ld.add_action(declare_multiOffsetRotateP)
    ld.add_action(declare_multiOffsetRotateY)
    ld.add_action(multi_transform_node)

    ld.add_action(OpaqueFunction(function=get_id_map_trans_publisher, 
                                 args=[[multiOffsetPositionX, multiOffsetPositionY, multiOffsetPositionZ, multiOffsetRotateY, multiOffsetRotateP, multiOffsetRotateR], robot_id]))
    
    return ld
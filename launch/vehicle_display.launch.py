import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_path = 'urdf/vehicle.urdf'
    robot_name_in_model = 'lognav_vehicle'

    pkg_share = launch_ros.substitutions.FindPackageShare(package='lognav_vehicle').find('lognav_vehicle')
    default_model_path = os.path.join(pkg_share, 'urdf/vehicle.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_lognav_gazebo = get_package_share_directory('lognav_vehicle')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ###############
    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "lognav_vehicle"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_lognav_gazebo , 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'
    ##########33

    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.33'
    spawn_yaw_val = '0.0'

    
    # print('aqui-----'+ pkg_lognav_gazebo)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'pause': 'true', 'use_sim_time': 'true'}.items()

    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('joint_gui')),
        parameters=[{'use_sim_time': use_sim_time}],
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('joint_gui')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='world',
            default_value=[os.path.join(pkg_lognav_gazebo, 'worlds', 'empty.world'), ''],
            description='Full path to the world model file to load'),
        DeclareLaunchArgument(
            name='joint_gui', 
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'),
        DeclareLaunchArgument(
            name='urdf_model', 
            default_value=default_urdf_model_path, 
            description='Absolute path to robot urdf file'),
        gazebo,
        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', robot_name_in_model, 
                        '-topic', 'robot_description',
                            '-x', spawn_x_val,
                            '-y', spawn_y_val,
                            '-z', spawn_z_val,
                            '-Y', spawn_yaw_val],
                            output='screen'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        # rviz_node,   
    ])

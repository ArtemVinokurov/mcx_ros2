import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, event, event_handlers
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown, RegisterEventHandler
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
import yaml
from webots_ros2_driver.webots_controller import WebotsController
from launch.event_handlers import OnProcessExit
from webots_ros2_driver.webots_launcher import WebotsLauncher


def load_yaml(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)
    print(absolute_file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print("error yaml file")
        return None

def load_file(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        print("error load file")
        return None
    

def generate_launch_description():

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    robot_ip_parameter_name = 'robot_ip'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
        
    manipulator_urdf_file = os.path.join(get_package_share_directory('moveit_config'), 'urdf', 'moveit_description.urdf.xacro')
    
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', manipulator_urdf_file])  
    
    
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file(
        'manipulator_description', 'resource', 'moveit2', 'pr15.srdf'
    )


    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'manipulator_description', 'resource', 'moveit2', 'kinematics.yaml'
    )

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.01,
        }
    }


    joint_limits_yaml = load_yaml(
        'manipulator_description', 'resource', 'moveit2', 'joint_limits.yaml'
    )
    
    ompl_planning_yaml = load_yaml(
        'manipulator_description', 'resource/moveit2/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'moveit_config', 'config', 'moveit_controllers.yaml'
    )

    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
    }

    trajectory_execution = {
        'allow_trajectory_execution': True,
        'allow_goal_duration_margin': True,
        'allow_start_tolerance': True,
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    use_sim_time = {'use_sim_time': False}

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            joint_limits_yaml,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            use_sim_time
        ],

    )

    rviz_base = os.path.join(get_package_share_directory('manipulator_description'), 'resource', 'moveit2')
    rviz_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            use_sim_time
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, use_sim_time], 
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    # load_controllers = []
    # for controller in ['mcx_arm_controller', 'mcx_joint_state_broadcaster']:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=['ros2 run controller_manager spawner.py {}'.format(controller)],
    #             shell=True,
    #             output='screen',
    #         )
    #     ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mcx_joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mcx_arm_controller", "--controller-manager", "/controller_manager"]
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration('db')
    mongodb_server_node = Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        parameters=[
            {'warehouse_port': 33829},
            {'warehouse_host': 'localhost'},
            {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'},
        ],
        output='screen',
        condition=IfCondition(db_config)
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='mcx_joint_state_publisher',
        parameters=[
            {'source_list': ['joint_states'], 'rate': 33}],
    )

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value='192.168.2.100',
        description='Hostname or IP address of the robot.')

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )


    return LaunchDescription([
        robot_arg,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        rviz_node,
        joint_state_publisher
        # delay_joint_state_broadcaster_after_robot_controller_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner
    ])
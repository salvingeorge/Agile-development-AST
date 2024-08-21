from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():

    # Gazebo world configuration
    gazebo_world_launch_arg = DeclareLaunchArgument(
        name="world",
        default_value="square_room.world",
        description="World model to be used in simulation"
    )

    # World path setup
    world_path = PathJoinSubstitution(
        [FindPackageShare("robile_gazebo"), "platform_independent", LaunchConfiguration("world")]
    )

    # Launch gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])
        ]),
        launch_arguments={"world": world_path}.items()
    )

    # Launch gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])
        ])
    )

    # Robot 1 setup
    path_robot_urdf = PathJoinSubstitution([FindPackageShare("robile_description"), "gazebo", "gazebo_robile_laser_scanner.xacro"])
    cmd_xacro_robot1 = Command(["xacro ", path_robot_urdf, " platform_config:=4_wheel_config ", "movable_joints:=False"])

    robot1_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        emulate_tty=True,
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(cmd_xacro_robot1, value_type=str)
        }],
        arguments=[path_robot_urdf]
    )

    spawn_robot1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robile1",
            "-x", "0.0", "-y", "0.0", "-z", "0.0"
        ]
    )

    # Robot 2 setup
    cmd_xacro_robot2 = Command(["xacro ", path_robot_urdf, " platform_config:=4_wheel_config ", "movable_joints:=False"])

    robot2_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        emulate_tty=True,
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(cmd_xacro_robot2, value_type=str)
        }],
        arguments=[path_robot_urdf]
    )

    spawn_robot2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robile2",
            "-x", "1.0", "-y", "0.0", "-z", "0.0"
        ]
    )

    # Static transform for both robots
    static_transform_robot1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"]
    )

    static_transform_robot2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"]
    )

    # Control nodes for both robots to listen to the same cmd_vel topic
    robot1_velocity_controller = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot_controller_1',
        output='screen',
        parameters=[{
            "use_sim_time": True
        }],
        remappings=[('/cmd_vel', '/cmd_vel')]  # Both robots will listen to the same /cmd_vel topic
    )

    robot2_velocity_controller = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot_controller_2',
        output='screen',
        parameters=[{
            "use_sim_time": True
        }],
        remappings=[('/cmd_vel', '/cmd_vel')]  # Both robots will listen to the same /cmd_vel topic
    )

    return LaunchDescription([
        # launch parameters declaration
        gazebo_world_launch_arg,

        # Launch and nodes
        gazebo_server,
        gazebo_client,

        # Robot 1
        robot1_state_publisher,
        spawn_robot1,
        static_transform_robot1,

        # Robot 2
        robot2_state_publisher,
        spawn_robot2,
        static_transform_robot2,

        # Controllers (both robots listen to the same /cmd_vel)
        robot1_velocity_controller,
        robot2_velocity_controller,
    ])
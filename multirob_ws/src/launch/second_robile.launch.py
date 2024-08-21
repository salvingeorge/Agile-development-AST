# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    gazebo_world_launch_arg = DeclareLaunchArgument(
            name = "world",
            default_value = "square_room.world",
            description = "World model to be used in simulation"
    )


    """
    Launch gazebo server
    """
    world_path = PathJoinSubstitution(
           [FindPackageShare("robile_gazebo").find("launch"), 
            "platform_independent",
            LaunchConfiguration("world")]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzserver.launch.py"
                ])
            ]),
        launch_arguments={
            "world": world_path,
            }.items()
        )

    """
    Launch gazebo client
    
    omg ros2 this is horrible !!
    why so complicated to launch stuff
    """

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzclient.launch.py"
                ])
            ])
        )

    # include another launch file
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('demo_nodes_cpp'),
                'launch/topics/talker_listener.launch.py'))
    )
    # include another launch file in the chatter_ns namespace
    launch_include_with_namespace = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('chatter_ns')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('demo_nodes_cpp'),
                        'launch/topics/talker_listener.launch.py'))
            ),
        ]
    )

    path_robot_urdf = PathJoinSubstitution([
        FindPackageShare("robile_description"),
        "gazebo",
        "gazebo_robile_laser_scanner.xacro"
        ])

    cmd_xacro = Command(["xacro ", path_robot_urdf, " platform_config:=4_wheel_config ", "movable_joints:=False"])

    joint_state_publisher = Node(package="joint_state_publisher",
                                 executable="joint_state_publisher",
                                 name="joint_state_publisher"
                                 )

    robot_state_publisher = Node(package="robot_state_publisher",
                                 executable="robot_state_publisher",
                                 name="robot_state_publisher",
                                 emulate_tty=True,
                                 output="screen",
                                 parameters=[{
                                     "use_sim_time": True,
                                     "robot_description": ParameterValue(cmd_xacro, value_type=str)
                                     }],
                                 arguments=[path_robot_urdf]
                                 )
    
    spawn_robot1 = Node(package="gazebo_ros",
                        namespace="robile1",
                       executable="spawn_entity.py",
                       arguments=[
                           "-topic", "robot_description",
                           "-entity", "robile"
                           ]
                       )
    

    # # start a turtlesim_node in the turtlesim1 namespace
    # turtlesim_node = Node(
    #         package='turtlesim',
    #         namespace='turtlesim1',
    #         executable='turtlesim_node',
    #         name='sim'
    #     )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters

    spawn_robot2 = Node(package="gazebo_ros",
                        namespace="robile2"
                       executable="spawn_entity.py",
                       arguments=[
                           "-topic", "robot_description",
                           "-entity", "robile"
                           ]
    )

    static_transform = Node(package="tf2_ros",
                            executable="static_transform_publisher",
                            output="screen",
                            # arguments order: x y z yaw pitch roll frame_id child_frame
                            arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"])
    
    turtlesim_node_with_parameters = Node(
            package='gazebo_ros',
            namespace='robile2',
            executable='spawn_entity.py',
            # parameters=[{
            #     "background_r": LaunchConfiguration('background_r'),
            #     "background_g": LaunchConfiguration('background_g'),
            #     "background_b": LaunchConfiguration('background_b'),
            # }]
        )

    # perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )

    return LaunchDescription([
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        chatter_ns_launch_arg,
        launch_include,
        launch_include_with_namespace,
        turtlesim_node,
        turtlesim_node_with_parameters,
        forward_turtlesim_commands_to_second_turtlesim_node,
    ])
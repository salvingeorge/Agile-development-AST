from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution

def generate_launch_description():
    
    """
    Gazebo world configuration
    """

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
            LaunchConfiguration("world.launch")]
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

    path_robot_urdf = PathJoinSubstitution([
        FindPackageShare("robile_description"),
        "gazebo",
        "gazebo_robile_laserscanner_camera.xacro"
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

    """
    Spawn robot in gazebo

    entity: robile
    topic: robot_description
    """
    spawn_robot = Node(package="gazebo_ros",
                       executable="spawn_entity.py",
                       arguments=[
                           "-topic", "robot_description",
                           "-entity", "robile"
                           ]
                       )

    """
    Static transfrom is require for connect base_link to base_footprint

    in the robile description base_link is at floor level
    so no transformation is require, but base_footprint need to be published
    """
    static_transform = Node(package="tf2_ros",
                            executable="static_transform_publisher",
                            output="screen",
                            # arguments order: x y z yaw pitch roll frame_id child_frame
                            arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"]
                            )


    return LaunchDescription([
        # launch parameters declaration
        gazebo_world_launch_arg,

        # Launch and nodes
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        static_transform,
    ])
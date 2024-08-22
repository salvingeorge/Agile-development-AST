import os
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_testing.asserts import assertInStdout


@pytest.mark.launch_test
def generate_test_description():

    # World path setup
    world_path = PathJoinSubstitution(
        [FindPackageShare("robile_gazebo"), "platform_independent", LaunchConfiguration("world")]
    )

    # Include the launch file to be tested
    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare("your_package_name").find("your_package_name"),
                'launch',
                'your_launch_file_name.launch.py'
            )
        ]),
        launch_arguments={"world": world_path}.items()
    )

    return LaunchDescription([
        launch_file,
        launch_testing.actions.ReadyToTest()
    ])


@pytest.mark.launch_test
def test_gazebo_launches(gazebo_launch_action, proc_output):
    """Test that Gazebo server and client nodes launch successfully."""
    # Check if gazebo server and client have started
    proc_output.assertWaitFor(
        'gzserver.launch.py', 
        timeout=10.0,
    )
    proc_output.assertWaitFor(
        'gzclient.launch.py', 
        timeout=10.0,
    )


@pytest.mark.launch_test
def test_robot1_launches(robot1_state_publisher, proc_output):
    """Test that Robot 1 state publisher starts successfully."""
    proc_output.assertWaitFor(
        'robot_state_publisher', 
        timeout=10.0,
    )


@pytest.mark.launch_test
def test_robot2_launches(robot2_state_publisher, proc_output):
    """Test that Robot 2 state publisher starts successfully."""
    proc_output.assertWaitFor(
        'robot_state_publisher', 
        timeout=10.0,
    )


@pytest.mark.launch_test
def test_static_transform_publishers(static_transform_robot1, static_transform_robot2, proc_output):
    """Test that static transform publishers for both robots are running."""
    proc_output.assertWaitFor(
        'static_transform_publisher', 
        timeout=10.0,
    )


@pytest.mark.launch_test
def test_velocity_controllers(robot1_velocity_controller, robot2_velocity_controller, proc_output):
    """Test that velocity controllers are properly set for both robots."""
    proc_output.assertWaitFor(
        'spawn_robot_controller_1', 
        timeout=10.0,
    )
    proc_output.assertWaitFor(
        'spawn_robot_controller_2', 
        timeout=10.0,
    )


if __name__ == '__main__':
    pytest.main()

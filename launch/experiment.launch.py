from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "experiment", package='rmd_test', executable='time_publisher'),
        launch_ros.actions.Node(
            namespace= "experiment", package='rmd_test', executable='trajectory_generator'),
        launch_ros.actions.Node(
            namespace= "experiment", package='rmd_test', executable='controller'),
    ])
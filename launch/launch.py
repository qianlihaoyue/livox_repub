import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='livox_repub',
            executable='livox_repub',
            name='livox_repub',
            output='screen',
            parameters=[
                {
                    'lid_topic': '/livox/lidar'
                },
                {
                    'pcl_topic': '/livox_pcl'
                }
            ]
        )
    ])
    return ld

